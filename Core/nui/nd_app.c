#include "nd_app.h"


#include "ui_conf.h"
#include "ui_types.h"
#include "ui_time.h"
#include "ui_packets.h"
#include "ui_rf_plan_kr920.h"
#include "ui_lpm.h"
#include "ui_uart.h"
#include "ui_ble.h"
#include "ui_gpio.h"
#include "ui_fault.h"
#include "ui_radio.h"

#include "nd_sensors.h"

#include "stm32_timer.h"
#include "stm32_seq.h"
#include "radio.h"

#include "main.h"

#include <string.h>
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* Node 상태                                                                  */
/* -------------------------------------------------------------------------- */
typedef enum
{
    ND_STATE_IDLE = 0,
    ND_STATE_RX_BEACON,
    ND_STATE_TX_DATA,
} ND_State_t;

static ND_State_t s_state = ND_STATE_IDLE;
static bool s_inited = false;

/* 타이머 */
static UTIL_TIMER_Object_t s_tmr_boot_listen;
static UTIL_TIMER_Object_t s_tmr_beacon_sched;
static UTIL_TIMER_Object_t s_tmr_sensor_sched;
static UTIL_TIMER_Object_t s_tmr_tx_sched;
static UTIL_TIMER_Object_t s_tmr_led1_pulse;

static volatile uint32_t s_evt_flags = 0;
#define ND_EVT_BOOT_LISTEN_START   (1u << 0)
#define ND_EVT_BEACON_LISTEN_START (1u << 1)
#define ND_EVT_SENSOR_START        (1u << 2)
#define ND_EVT_TX_START            (1u << 3)

static bool     s_beacon_ok = false;         /* 최근 비콘 수신 성공 여부 */
static uint16_t s_beacon_cnt = 0;            /* 비콘 수신 카운터 */
static bool     s_test_mode = false;
/* 최초 유효 비콘을 받은 뒤에만 측정/송신 동작 허용 */
static bool     s_runtime_enabled = false;

static ND_SensorResult_t s_last_sensor;
static bool s_sensor_ready = false;
static uint8_t s_node_tx_payload[UI_NODE_PAYLOAD_LEN];

/* 부팅 후 초기 6분은 6초 RX window를 반복해서 beacon 시간을 맞춘다. */
#define ND_BOOT_RX_WINDOW_MS          (6000u)
/*
 * 운영 중 beacon re-sync window:
 *  - beacon은 00/02/04초에 나가므로, 분/5분 경계보다 약간 일찍 깨어나야 한다.
 *  - 6초 측정 시각과 겹치지 않도록 0.5초 일찍 시작해서 6.0초 동안만 듣는다.
 */
#define ND_BEACON_EARLY_WAKE_MS       (1000u)
#define ND_BEACON_TRACK_WINDOW_MS     (7500u)
/* Beacon payload(21B, SF11, BW125) is received roughly 0.6~0.7s after TX start.
 * Apply centi-second correction so the next 1-minute beacon window opens on time. */
#define ND_BEACON_RX_TIME_CORR_CENTI  (74u)

static bool     s_boot_listen_active = false;
static uint32_t s_boot_listen_deadline_ms = 0u;
static uint32_t s_rx_window_deadline_ms = 0u;

static bool prv_radio_ready_for_tx(void)
{
    if ((Radio.SetChannel == NULL) || (Radio.Send == NULL) || (Radio.Sleep == NULL))
    {
        UI_FAULT_MARK("ND_RADIO_NULL",
                      (Radio.SetChannel == NULL) ? 1u : 0u,
                      (Radio.Send == NULL) ? 1u : 0u);
        return false;
    }
    return true;
}

static bool prv_radio_ready_for_rx(void)
{
    if ((Radio.SetChannel == NULL) || (Radio.Rx == NULL) || (Radio.Sleep == NULL))
    {
        UI_FAULT_MARK("ND_RADIO_NULL",
                      (Radio.SetChannel == NULL) ? 1u : 0u,
                      (Radio.Rx == NULL) ? 1u : 0u);
        return false;
    }
    return true;
}

/* LED1 */
static void prv_led1(bool on)
{
#if UI_HAVE_LED1
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}

static void prv_led1_pulse_off_cb(void *context)
{
    (void)context;
    prv_led1(false);
}

static void prv_led1_pulse_10ms(void)
{
    prv_led1(true);
    (void)UTIL_TIMER_Stop(&s_tmr_led1_pulse);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_led1_pulse, 10u);
    (void)UTIL_TIMER_Start(&s_tmr_led1_pulse);
}

/* -------------------------------------------------------------------------- */
/* 타이머 callback -> task                                                    */
/* -------------------------------------------------------------------------- */
static void prv_tmr_boot_cb(void *context)
{
    (void)context;
    UI_FAULT_MARK("ND_BOOT_CB", 0u, 0u);
    s_evt_flags |= ND_EVT_BOOT_LISTEN_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_tmr_beacon_cb(void *context)
{
    (void)context;
    UI_FAULT_MARK("ND_BCN_CB", 0u, 0u);
    s_evt_flags |= ND_EVT_BEACON_LISTEN_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_tmr_sensor_cb(void *context)
{
    (void)context;
    UI_FAULT_MARK("ND_SNS_CB", 0u, 0u);
    s_evt_flags |= ND_EVT_SENSOR_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_tmr_tx_cb(void *context)
{
    (void)context;
    UI_FAULT_MARK("ND_TX_CB", 0u, 0u);
    s_evt_flags |= ND_EVT_TX_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

/* -------------------------------------------------------------------------- */
/* 스케줄 계산                                                                */
/* -------------------------------------------------------------------------- */
static void prv_apply_setting_ascii(const uint8_t setting_ascii[3])
{
    /*
     * 노드 규칙:
     *  - "01M" 이면 1분 테스트 모드
     *  - 그 외는 정상 모드(설정 주기)
     *
     * 비콘을 한 번만 받아도 이후에는 로컬 설정으로 계속 동작한다.
     * 이후 비콘을 잠시 놓쳐도 직전 설정을 유지한다.
     */
    uint8_t d0 = setting_ascii[0];
    uint8_t d1 = setting_ascii[1];
    char unit  = (char)setting_ascii[2];

    if ((d0 < '0') || (d0 > '9') || (d1 < '0') || (d1 > '9') || ((unit != 'M') && (unit != 'H')))
    {
        UI_SetSetting(0u, 'H');
        s_test_mode = false;
        return;
    }

    uint8_t value = (uint8_t)((d0 - '0') * 10u + (d1 - '0'));
    UI_SetSetting(value, unit);
    s_test_mode = ((value == 1u) && (unit == 'M'));
}

static void prv_refresh_mode_from_config(void)
{
    const UI_Config_t* cfg = UI_GetConfig();
    s_test_mode = ((cfg->setting_value == 1u) && (cfg->setting_unit == 'M'));
}

static uint32_t prv_get_setting_cycle_sec(void)
{
    const UI_Config_t* cfg = UI_GetConfig();

    if ((cfg->setting_value == 0u) || ((cfg->setting_unit != 'M') && (cfg->setting_unit != 'H')))
    {
        return 0u;
    }

    if (cfg->setting_unit == 'M')
    {
        return (uint32_t)cfg->setting_value * 60u;
    }

    return (uint32_t)cfg->setting_value * 3600u;
}

static uint32_t prv_get_normal_cycle_sec(void)
{
    uint32_t cycle_sec = prv_get_setting_cycle_sec();

    if (cycle_sec == 0u)
    {
        return 3600u;
    }

    /* 정상 모드는 5분 기준으로 비콘을 맞추므로, 더 짧은 값은 5분으로 올려서 사용한다. */
    if (cycle_sec < UI_BEACON_PERIOD_S)
    {
        return UI_BEACON_PERIOD_S;
    }

    return cycle_sec;
}

static uint32_t prv_get_beacon_interval_sec(void)
{
    return s_test_mode ? 60u : UI_BEACON_PERIOD_S;
}

static uint64_t prv_next_event_centi(uint64_t now_centi, uint32_t interval_sec, uint32_t offset_sec)
{
    uint32_t now_sec = (uint32_t)(now_centi / 100u);
    uint32_t centi   = (uint32_t)(now_centi % 100u);

    uint32_t cand_sec = now_sec + ((centi == 0u) ? 0u : 1u);

    if (interval_sec == 0u)
    {
        return (uint64_t)(cand_sec + offset_sec) * 100u;
    }

    uint32_t rem = cand_sec % interval_sec;
    uint32_t next_sec;

    if (rem <= offset_sec)
        next_sec = cand_sec - rem + offset_sec;
    else
        next_sec = cand_sec - rem + interval_sec + offset_sec;

    return (uint64_t)next_sec * 100u;
}

static void prv_start_beacon_rx(uint32_t window_ms);
static void prv_schedule_beacon_window(void);
static void prv_schedule_sensor_and_tx(void);

static void prv_stop_sensor_and_tx_timers(void)
{
    (void)UTIL_TIMER_Stop(&s_tmr_sensor_sched);
    (void)UTIL_TIMER_Stop(&s_tmr_tx_sched);
}

static uint32_t prv_ms_until(uint32_t deadline_ms)
{
    uint32_t now = UTIL_TIMER_GetCurrentTime();
    return ((int32_t)(deadline_ms - now) > 0) ? (deadline_ms - now) : 0u;
}

static void prv_begin_boot_listen(void)
{
    s_boot_listen_active = true;
    s_boot_listen_deadline_ms = UTIL_TIMER_GetCurrentTime() + UI_ND_BOOT_LISTEN_MS;

    uint32_t first_ms = UI_ND_BOOT_LISTEN_MS;
    if (first_ms > ND_BOOT_RX_WINDOW_MS)
    {
        first_ms = ND_BOOT_RX_WINDOW_MS;
    }
    if (first_ms == 0u)
    {
        first_ms = 1u;
    }

    UI_FAULT_MARK("ND_BOOT_RX", first_ms, s_boot_listen_deadline_ms);
    prv_start_beacon_rx(first_ms);
}

static bool prv_restart_current_rx_window(void)
{
    uint32_t remain_ms = prv_ms_until(s_rx_window_deadline_ms);
    if (remain_ms < 20u)
    {
        return false;
    }

    UI_FAULT_MARK("ND_RX_RETRY", remain_ms, s_state);
    prv_start_beacon_rx(remain_ms);
    return true;
}

static void prv_continue_boot_listen_or_schedule(void)
{
    if (s_boot_listen_active)
    {
        uint32_t remain_ms = prv_ms_until(s_boot_listen_deadline_ms);
        if (remain_ms > 0u)
        {
            uint32_t next_ms = remain_ms;
            if (next_ms > ND_BOOT_RX_WINDOW_MS)
            {
                next_ms = ND_BOOT_RX_WINDOW_MS;
            }
            if (next_ms == 0u)
            {
                next_ms = 1u;
            }

            UI_FAULT_MARK("ND_BOOT_NEXT", next_ms, remain_ms);
            prv_start_beacon_rx(next_ms);
            return;
        }

        s_boot_listen_active = false;
        s_boot_listen_deadline_ms = 0u;
    }

    prv_schedule_beacon_window();
    if (s_runtime_enabled)
    {
        prv_schedule_sensor_and_tx();
    }
    else
    {
        prv_stop_sensor_and_tx_timers();
    }
}

static void prv_start_beacon_rx(uint32_t window_ms)
{
    UI_FAULT_MARK("ND_BCN_RX", window_ms, s_state);
    /* radio 사용 중이면 무시 */
    if (s_state != ND_STATE_IDLE) return;

    if (window_ms == 0u)
    {
        window_ms = 1u;
    }

    if (!prv_radio_ready_for_rx()) return;
    if (!UI_Radio_PrepareRx(UI_BEACON_PAYLOAD_LEN))
    {
        UI_FAULT_MARK("ND_BCN_CFGF", window_ms, 0u);
        return;
    }

    /* beacon window 시작 시, 이번 window의 결과가 곧 s_beacon_ok가 됨 */
    s_beacon_ok = false;
    s_rx_window_deadline_ms = UTIL_TIMER_GetCurrentTime() + window_ms;

    UI_LPM_LockStop();
    s_state = ND_STATE_RX_BEACON;

    UI_FAULT_MARK("ND_BCN_SET0", UI_RF_GetBeaconFreqHz(), window_ms);
    Radio.SetChannel(UI_RF_GetBeaconFreqHz());
    UI_FAULT_MARK("ND_BCN_SET1", UI_RF_GetBeaconFreqHz(), window_ms);
    UI_FAULT_MARK("ND_BCN_CALL0", UI_RF_GetBeaconFreqHz(), window_ms);
    Radio.Rx(window_ms);
    UI_FAULT_MARK("ND_BCN_CALL1", UI_RF_GetBeaconFreqHz(), window_ms);
}

static void prv_schedule_beacon_window(void)
{
    /*
     * 운영 중 beacon re-sync:
     *  - 정상 모드: 5분마다
     *  - 테스트 01M: 1분마다
     *  - gw0/1/2가 00/02/04초에 보내므로, 경계보다 조금 일찍 깨어나서 6초만 듣는다.
     */
    uint64_t now = UI_Time_NowCenti2016();
    uint32_t interval_sec = prv_get_beacon_interval_sec();
    uint64_t next = prv_next_event_centi(now, interval_sec, 0u);

    uint64_t delta_centi = (next > now) ? (next - now) : 1u;
    uint32_t delta_ms = (uint32_t)(delta_centi * 10u);

    if (delta_ms > ND_BEACON_EARLY_WAKE_MS)
    {
        delta_ms -= ND_BEACON_EARLY_WAKE_MS;
    }
    else
    {
        delta_ms = 1u;
    }

    UI_FAULT_MARK("ND_BCN_SCH", interval_sec, delta_ms);
    (void)UTIL_TIMER_Stop(&s_tmr_beacon_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_beacon_sched, delta_ms);
    (void)UTIL_TIMER_Start(&s_tmr_beacon_sched);
}

static void prv_schedule_sensor_and_tx(void)
{
    if (!s_runtime_enabled)
    {
        prv_stop_sensor_and_tx_timers();
        s_sensor_ready = false;
        return;
    }

    prv_refresh_mode_from_config();

    const UI_Config_t* cfg = UI_GetConfig();
    uint8_t node = cfg->node_num;

    /* 테스트 모드에서 노드 10개 제한 */
    if (s_test_mode && node >= UI_TESTMODE_MAX_NODES)
    {
        return;
    }

    uint64_t now = UI_Time_NowCenti2016();

    uint32_t period = s_test_mode ? 60u : prv_get_normal_cycle_sec();
    uint32_t sensor_off = s_test_mode ? UI_ND_SENSOR_START_S_TEST : UI_ND_SENSOR_START_S_NORMAL;

    /* TX offset:
     *  - 01M test mode: 기존 30초 슬롯 유지
     *  - normal mode  : 설정 주기 +01분부터 node별 2초 슬롯 송신 */
    uint32_t base_tx = s_test_mode ? 30u : 60u;
    uint32_t tx_off = base_tx + (uint32_t)node * 2u;

    uint64_t next_sensor = prv_next_event_centi(now, period, sensor_off);
    uint64_t next_tx     = prv_next_event_centi(now, period, tx_off);

    /* 센서/송신 예약 */
    uint32_t ds_ms = (uint32_t)((next_sensor > now) ? ((next_sensor - now) * 10u) : 1u);
    uint32_t dt_ms = (uint32_t)((next_tx > now) ? ((next_tx - now) * 10u) : 1u);

    (void)UTIL_TIMER_Stop(&s_tmr_sensor_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_sensor_sched, ds_ms);
    (void)UTIL_TIMER_Start(&s_tmr_sensor_sched);

    (void)UTIL_TIMER_Stop(&s_tmr_tx_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_tx_sched, dt_ms);
    (void)UTIL_TIMER_Start(&s_tmr_tx_sched);
}

/* -------------------------------------------------------------------------- */
/* OP_KEY 처리 (UI_Core hook override)                                        */
/* -------------------------------------------------------------------------- */
void UI_Hook_OnOpKeyPressed(void)
{
    /* 센서 측정 -> BLE ON -> ASCII 전송 */
    prv_led1(true);

    ND_SensorResult_t r;
    (void)ND_Sensors_MeasureAll(&r);

    UI_BLE_EnableForMs(UI_BLE_ACTIVE_MS);

    /* <날짜:X,Y,Z,ADC VALUE, PULSE COUNT> */
    char ts[48];
    UI_Time_FormatNow(ts, sizeof(ts));

    char msg[160];
    uint32_t pulse = r.pulse_cnt;

    (void)snprintf(msg, sizeof(msg),
                   "<%s:%d,%d,%d,%u,%lu>\r\n",
                   ts,
                   (int)r.x, (int)r.y, (int)r.z,
                   (unsigned)r.adc,
                   (unsigned long)pulse);

    UI_UART_SendString(msg);

    prv_led1(false);
}

/* -------------------------------------------------------------------------- */
/* Task main                                                                   */
/* -------------------------------------------------------------------------- */
void ND_App_Process(void)
{
    if (!s_inited)
    {
        return;
    }


    uint32_t ev = s_evt_flags;
    s_evt_flags = 0;

    UI_FAULT_MARK("ND_PROC", ev, s_state);

    if ((ev & ND_EVT_BOOT_LISTEN_START) != 0u)
    {
        /* 부팅 후 6분은 6초 window를 반복해서 beacon을 잡는다. */
        prv_begin_boot_listen();
        return;
    }

    if ((ev & ND_EVT_BEACON_LISTEN_START) != 0u)
    {
        /* 설정 모드에 맞춰 다음 beacon re-sync window 실행 */
        prv_start_beacon_rx(ND_BEACON_TRACK_WINDOW_MS);
        return;
    }

    if ((ev & ND_EVT_SENSOR_START) != 0u)
    {
        if (!s_runtime_enabled)
        {
            UI_FAULT_MARK("ND_SNS_SKIP", s_boot_listen_active ? 1u : 0u, UI_Time_IsValid() ? 1u : 0u);
            s_sensor_ready = false;
            return;
        }

        s_sensor_ready = ND_Sensors_MeasureAll(&s_last_sensor);
        return;
    }

    if ((ev & ND_EVT_TX_START) != 0u)
    {
        if (!s_runtime_enabled)
        {
            UI_FAULT_MARK("ND_TX_SKIP", s_boot_listen_active ? 1u : 0u, UI_Time_IsValid() ? 1u : 0u);
            return;
        }

        if (!s_sensor_ready) return;

        if (s_state != ND_STATE_IDLE) return;

        /* 데이터 송신 */
        const UI_Config_t* cfg = UI_GetConfig();
        uint32_t now_sec = UI_Time_NowSec2016();

        uint32_t hop_period = s_test_mode ? 60u : prv_get_normal_cycle_sec();
        uint32_t freq = UI_RF_GetDataFreqHz(now_sec, hop_period, 0);

        UI_NodeData_t nd = {0};
        nd.node_num   = cfg->node_num;
        memcpy(nd.net_id, cfg->net_id, UI_NET_ID_LEN);
        nd.batt_lvl  = s_last_sensor.batt_lvl;
        nd.temp_c     = s_last_sensor.temp_c;
        nd.beacon_cnt = s_beacon_cnt;
        nd.x          = s_last_sensor.x;
        nd.y          = s_last_sensor.y;
        nd.z          = s_last_sensor.z;
        nd.adc        = s_last_sensor.adc;
        nd.pulse_cnt  = s_last_sensor.pulse_cnt;

        (void)UI_Pkt_BuildNodeData(s_node_tx_payload, &nd);

        if (!prv_radio_ready_for_tx())
        {
            return;
        }
        if (!UI_Radio_PrepareTx(UI_NODE_PAYLOAD_LEN))
        {
            UI_FAULT_MARK("ND_TX_CFGF", freq, UI_NODE_PAYLOAD_LEN);
            return;
        }

        UI_LPM_LockStop();
        s_state = ND_STATE_TX_DATA;

        UI_FAULT_MARK("ND_TX_SET0", freq, UI_NODE_PAYLOAD_LEN);
        Radio.SetChannel(freq);
        UI_FAULT_MARK("ND_TX_SET1", freq, UI_NODE_PAYLOAD_LEN);
        prv_led1_pulse_10ms();
        UI_FAULT_MARK("ND_TX_SEND0", freq, UI_NODE_PAYLOAD_LEN);
        Radio.Send(s_node_tx_payload, UI_NODE_PAYLOAD_LEN);
        UI_FAULT_MARK("ND_TX_SEND1", freq, UI_NODE_PAYLOAD_LEN);
        return;
    }

}

static void ND_TaskMain(void)
{
    /* 멀티 task 모드에서만 등록/호출됨 */
    ND_App_Process();
}


/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */
void ND_App_Init(void)
{
    if (s_inited)
    {
        return;
    }
    /* Task 등록 (task bit이 충분한 경우에만 분리) */
#if (UI_USE_SEQ_MULTI_TASKS == 1u)
    UTIL_SEQ_RegTask(UI_TASK_BIT_ND_MAIN, 0, ND_TaskMain);
#endif

    (void)UTIL_TIMER_Create(&s_tmr_boot_listen, 100u, UTIL_TIMER_ONESHOT, prv_tmr_boot_cb, NULL);
    (void)UTIL_TIMER_Create(&s_tmr_beacon_sched, 100u, UTIL_TIMER_ONESHOT, prv_tmr_beacon_cb, NULL);
    (void)UTIL_TIMER_Create(&s_tmr_sensor_sched, 100u, UTIL_TIMER_ONESHOT, prv_tmr_sensor_cb, NULL);
    (void)UTIL_TIMER_Create(&s_tmr_tx_sched, 100u, UTIL_TIMER_ONESHOT, prv_tmr_tx_cb, NULL);
    (void)UTIL_TIMER_Create(&s_tmr_led1_pulse, 10u, UTIL_TIMER_ONESHOT, prv_led1_pulse_off_cb, NULL);

    UI_FAULT_MARK("ND_INIT", 0u, 0u);
    ND_Sensors_Init();

    s_state = ND_STATE_IDLE;
    s_beacon_ok = false;
    s_beacon_cnt = 0;
    s_test_mode = false;
    s_runtime_enabled = false;
    s_sensor_ready = false;
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    s_rx_window_deadline_ms = 0u;
    UI_Radio_MarkRecoverNeeded();

    s_inited = true;

    prv_refresh_mode_from_config();
    prv_stop_sensor_and_tx_timers();

    /* 부팅 후 6분은 비콘만 대기 */
    (void)UTIL_TIMER_SetPeriod(&s_tmr_boot_listen, 10u);
    (void)UTIL_TIMER_Start(&s_tmr_boot_listen);
}

/* -------------------------------------------------------------------------- */
/* Radio event handlers                                                       */
/* -------------------------------------------------------------------------- */
void ND_Radio_OnTxDone(void)
{
    UI_FAULT_MARK("ND_TXDONE", s_state, 0u);
    if (s_state == ND_STATE_TX_DATA)
    {
        UI_Radio_MarkRecoverNeeded();
        Radio.Sleep();
        s_state = ND_STATE_IDLE;
        UI_LPM_UnlockStop();

        /* 다음 스케줄 갱신 */
        prv_continue_boot_listen_or_schedule();
        prv_schedule_sensor_and_tx();
    }
}

void ND_Radio_OnTxTimeout(void)
{
    UI_FAULT_MARK("ND_TXTO", s_state, 0u);
    /*
     * subghz_phy_app.c의 OnTxTimeout()에서 호출.
     * TX timeout이 발생하면 Radio를 Sleep으로 보내고 상태를 복구해야
     * 불필요한 소비 전류가 줄어듭니다.
     */
    UI_Radio_MarkRecoverNeeded();
    Radio.Sleep();

    if (s_state != ND_STATE_IDLE)
    {
        s_state = ND_STATE_IDLE;
        UI_LPM_UnlockStop();
    }

    prv_continue_boot_listen_or_schedule();
    prv_schedule_sensor_and_tx();
}

void ND_Radio_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    (void)rssi;
    (void)snr;

    UI_FAULT_MARK("ND_RXDONE", size, s_state);
    if (s_state == ND_STATE_RX_BEACON)
    {
        bool beacon_valid = false;
        UI_Beacon_t b;

        if (size != UI_BEACON_PAYLOAD_LEN)
        {
            UI_FAULT_MARK("ND_BCN_LEN", size, UI_BEACON_PAYLOAD_LEN);
        }
        else if (UI_Pkt_ParseBeacon(payload, size, &b))
        {
            /* NETID 체크:
             *  - 초기에는 NETID가 기본값일 수 있으므로, 동일하면 OK
             *  - 다른 경우도 '새 NETID로 갱신'이 필요하면 아래 정책 변경 가능
             */
            const UI_Config_t* cfg = UI_GetConfig();
            if (memcmp(b.net_id, cfg->net_id, UI_NET_ID_LEN) == 0)
            {
                beacon_valid = true;

                /* 실제 비콘 수신 완료 시 LED1 10ms pulse */
                prv_led1_pulse_10ms();

                /*
                 * Beacon time correction:
                 *  - GW beacon time field is the TX start second (00/02/04).
                 *  - ND receives RxDone about 0.6~0.7 s later, so setting centi=0 makes
                 *    the local clock lag behind and the next 1-minute beacon window opens late.
                 *  - Apply a fixed centi correction so minute alignment stays locked.
                 */
                uint64_t prev_centi = UI_Time_IsValid() ? UI_Time_NowCenti2016() : 0u;
                uint32_t epoch_sec = UI_Time_Epoch2016_FromCalendar(&b.dt);
                uint64_t epoch_centi = (uint64_t)epoch_sec * 100u + (uint64_t)ND_BEACON_RX_TIME_CORR_CENTI;
                UI_FAULT_MARK("ND_BCN_TIME", (uint32_t)(prev_centi % 100u), ND_BEACON_RX_TIME_CORR_CENTI);
                UI_Time_SetEpochCenti2016(epoch_centi);

                /* setting 반영: 01M만 테스트 모드, 그 외는 정상 모드 */
                prv_apply_setting_ascii(b.setting_ascii);

                s_beacon_cnt++;
                s_beacon_ok = true;
                s_boot_listen_active = false;
                s_boot_listen_deadline_ms = 0u;
                s_runtime_enabled = true;
                s_sensor_ready = false;

                UI_FAULT_MARK("ND_SYNC_OK", s_beacon_cnt, UI_Time_NowCentiPart());

                /* 최초 유효 비콘 이후부터 측정/송신 스케줄 시작 */
                prv_schedule_beacon_window();
                prv_schedule_sensor_and_tx();
            }
            else
            {
                UI_FAULT_MARK("ND_NET_MIS", b.net_id[0], cfg->net_id[0]);
            }
        }
        else
        {
            UI_FAULT_MARK("ND_BCN_BAD", size, payload ? payload[0] : 0u);
        }

        UI_Radio_MarkRecoverNeeded();
        Radio.Sleep();
        s_state = ND_STATE_IDLE;
        UI_LPM_UnlockStop();

        if (!beacon_valid)
        {
            if (!prv_restart_current_rx_window())
            {
                prv_continue_boot_listen_or_schedule();
            }
        }
    }
}

void ND_Radio_OnRxTimeout(void)
{
    UI_FAULT_MARK("ND_RXTO", s_state, 0u);
    if (s_state == ND_STATE_RX_BEACON)
    {
        /* 이번 window에서는 beacon 수신 실패 */
        s_beacon_ok = false;

        UI_Radio_MarkRecoverNeeded();
        Radio.Sleep();
        s_state = ND_STATE_IDLE;
        UI_LPM_UnlockStop();

        /* 초기 6분 구간이면 계속 이어서 듣고, 그 외에는 다음 window 예약 */
        prv_continue_boot_listen_or_schedule();
    }
}

void ND_Radio_OnRxError(void)
{
    UI_FAULT_MARK("ND_RXERR", s_state, 0u);
    if (s_state == ND_STATE_RX_BEACON)
    {
        s_beacon_ok = false;

        UI_Radio_MarkRecoverNeeded();
        Radio.Sleep();
        s_state = ND_STATE_IDLE;
        UI_LPM_UnlockStop();

        prv_continue_boot_listen_or_schedule();
    }
}

/* -------------------------------------------------------------------------- */
/* UI_CMD hook overrides                                                      */
/* -------------------------------------------------------------------------- */
void UI_Hook_OnConfigChanged(void)
{
    /* 노드 번호/NETID/SETTING 변경 -> 비콘/운영 스케줄 재계산 */
    prv_refresh_mode_from_config();
    if (!s_boot_listen_active)
    {
        prv_schedule_beacon_window();
    }
    if (s_runtime_enabled)
    {
        prv_schedule_sensor_and_tx();
    }
    else
    {
        prv_stop_sensor_and_tx_timers();
    }
}

void UI_Hook_OnTimeChanged(void)
{
    /* 수동 시간 변경만으로는 운영 시작하지 않고, 최초 유효 비콘 이후에만 측정/송신 */
    if (!s_boot_listen_active)
    {
        prv_schedule_beacon_window();
    }
    if (s_runtime_enabled)
    {
        prv_schedule_sensor_and_tx();
    }
}

