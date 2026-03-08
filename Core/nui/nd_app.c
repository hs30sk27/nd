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
#include "ui_radio.h"
#include "ui_crc16.h"

#include "nd_sensors.h"

#include "stm32_timer.h"
#include "stm32_seq.h"
#include "radio.h"

#include "main.h"

extern RTC_HandleTypeDef hrtc;

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

typedef enum
{
    ND_RX_REASON_NONE = 0,
    ND_RX_REASON_BOOT,
    ND_RX_REASON_MAIN,
    ND_RX_REASON_REMINDER,
    ND_RX_REASON_SEARCH,
} ND_RxReason_t;

typedef enum
{
    ND_SYNC_BOOT_SEARCH = 0,
    ND_SYNC_LOCKED,
    ND_SYNC_HOLDOVER,
    ND_SYNC_UNSYNC_SEARCH,
} ND_SyncState_t;

static ND_State_t s_state = ND_STATE_IDLE;
static bool s_inited = false;

/* 타이머 */
static UTIL_TIMER_Object_t s_tmr_boot_listen;
static UTIL_TIMER_Object_t s_tmr_beacon_sched;
static UTIL_TIMER_Object_t s_tmr_reminder_sched;
static UTIL_TIMER_Object_t s_tmr_sensor_sched;
static UTIL_TIMER_Object_t s_tmr_tx_sched;
static UTIL_TIMER_Object_t s_tmr_led1_pulse;

static volatile uint32_t s_evt_flags = 0;
#define ND_EVT_BOOT_LISTEN_START      (1u << 0)
#define ND_EVT_BEACON_LISTEN_START    (1u << 1)
#define ND_EVT_SENSOR_START           (1u << 2)
#define ND_EVT_TX_START               (1u << 3)
#define ND_EVT_REMINDER_LISTEN_START  (1u << 4)

static bool     s_beacon_ok = false;         /* 최근 비콘 수신 성공 여부 */
static uint16_t s_beacon_cnt = 0;            /* 비콘 수신 카운터 */
static bool     s_test_mode = false;
static ND_SyncState_t s_sync_state = ND_SYNC_BOOT_SEARCH;
/* 최초 유효 비콘 이후에만 runtime을 켜고, miss가 누적되면 HOLDOVER/UNSYNC_SEARCH로 전이한다. */
static bool     s_runtime_enabled = false;
static uint8_t  s_beacon_miss_count = 0u;
static uint32_t s_last_beacon_anchor_sec = 0u;

static ND_SensorResult_t s_last_sensor;
static bool s_sensor_ready = false;
static uint8_t s_node_tx_payload[UI_NODE_PAYLOAD_LEN];
static uint32_t s_last_sensor_slot_id = 0xFFFFFFFFu;
static uint32_t s_last_tx_slot_id = 0xFFFFFFFFu;

#define ND_TX_IN_SLOT_DELAY_MS            (200u)

/* 부팅 후 초기 6분은 6초 RX window를 반복해서 beacon 시간을 맞춘다. */
#define ND_BOOT_RX_WINDOW_MS              (6000u)
/*
 * 운영 중 beacon re-sync window:
 *  - 기본 5분 모드가 기준이고, 1분/2분은 테스트 모드다.
 *  - GW beacon은 1회만 송신하므로, ND는 최근에 받은 beacon anchor를 기준으로
 *    다음 beacon 시각을 예측해 조금 일찍 깨어난다.
 *  - 비콘을 놓치면 early wake를 단계적으로 넓혀 HOLDOVER/UNSYNC_SEARCH에서도
 *    다음 주기 beacon을 저전력으로 재획득한다.
 */
#define ND_BEACON_EARLY_WAKE_MS           (1000u)
#define ND_BEACON_EARLY_WAKE_MS_2M        (500u)
#define ND_BEACON_EARLY_WAKE_STEP_MS      (500u)
#define ND_BEACON_EARLY_WAKE_MAX_MS       (2500u)
#define ND_BEACON_EARLY_WAKE_MAX_MS_2M    (1500u)
#define ND_BEACON_TAIL_MS_NORMAL          (5600u)
#define ND_BEACON_TAIL_MS_2M              (5800u)
/* Beacon payload(21B, SF11, BW125) is received roughly 0.6~0.7s after TX start.
 * Apply centi-second correction so the next beacon window opens on time. */
#define ND_BEACON_RX_TIME_CORR_CENTI      (74u)

static bool     s_boot_listen_active = false;
static uint32_t s_boot_listen_deadline_ms = 0u;
static uint32_t s_rx_window_deadline_ms = 0u;
static ND_RxReason_t s_rx_reason = ND_RX_REASON_NONE;

#define ND_GW_PHASE_SCAN_EVERY_N_CYCLES   (12u)
#define ND_GW_PHASE_SCAN_EXTRA_MS         (4500u)

static bool    s_force_gw_phase_scan = false;
static uint8_t s_gw_phase_scan_cycle_count = 0u;

#define ND_REMINDER_RX_WINDOW_MS_BASE      (2200u)
#define ND_REMINDER_RX_WINDOW_MS_MAX       (3600u)
#define ND_REMINDER_RX_WINDOW_STEP_MS      (200u)
#define ND_REMINDER_TX_GUARD_MS            (600u)
#define ND_SYNC_HOLDOVER_MAX_MISS          (2u)
#define ND_SEARCH_RX_WINDOW_MS             (5000u)
#define ND_SEARCH_SCAN_INTERVAL_MS_BASE    (47000u)
#define ND_SEARCH_SCAN_INTERVAL_MS_JITTER  (13000u)

#define ND_SYNC_BKP_MAGIC                  (0x4E445359u) /* 'NDSY' */
#define ND_SYNC_BKP_DR_MAGIC               (RTC_BKP_DR0)
#define ND_SYNC_BKP_DR_EPOCH               (RTC_BKP_DR1)
#define ND_SYNC_BKP_DR_META                (RTC_BKP_DR2)
#define ND_SYNC_BKP_DR_CRC                 (RTC_BKP_DR3)

static bool prv_radio_ready_for_tx(void)
{
    if ((Radio.SetChannel == NULL) || (Radio.Send == NULL) || (Radio.Sleep == NULL))
    {
        return false;
    }
    return true;
}

static bool prv_radio_ready_for_rx(void)
{
    if ((Radio.SetChannel == NULL) || (Radio.Rx == NULL) || (Radio.Sleep == NULL))
    {
        return false;
    }
    return true;
}

/* LED0 */
static void prv_led0(bool on)
{
#if defined(LED0_GPIO_Port) && defined(LED0_Pin)
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
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

static void prv_power_on_led_blink(void)
{
    for (uint32_t i = 0u; i < 2u; i++)
    {
        prv_led0(true);
        prv_led1(true);
        HAL_Delay(100u);
        prv_led0(false);
        prv_led1(false);
        HAL_Delay(100u);
    }
}

/* -------------------------------------------------------------------------- */
/* 타이머 callback -> task                                                    */
/* -------------------------------------------------------------------------- */
static void prv_tmr_boot_cb(void *context)
{
    (void)context;
    s_evt_flags |= ND_EVT_BOOT_LISTEN_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_tmr_beacon_cb(void *context)
{
    (void)context;
    s_evt_flags |= ND_EVT_BEACON_LISTEN_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_tmr_reminder_cb(void *context)
{
    (void)context;
    s_evt_flags |= ND_EVT_REMINDER_LISTEN_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_tmr_sensor_cb(void *context)
{
    (void)context;
    s_evt_flags |= ND_EVT_SENSOR_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_tmr_tx_cb(void *context)
{
    (void)context;
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

static bool prv_is_two_minute_mode_active(void)
{
    const UI_Config_t* cfg = UI_GetConfig();
    return ((cfg->setting_value == 2u) && (cfg->setting_unit == 'M'));
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

    /* SETTING:2M은 별도 모드로 취급한다.
     * - beacon listen : 2분마다 00초~05초 window
     * - sensor start  : +06초
     * - data tx       : +30초부터 node별 2초 슬롯
     */
    if (prv_is_two_minute_mode_active())
    {
        return 120u;
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
    if (s_test_mode)
    {
        return 60u;
    }
    if (prv_is_two_minute_mode_active())
    {
        return 120u;
    }
    return UI_BEACON_PERIOD_S;
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

static void prv_start_beacon_rx(uint32_t window_ms, ND_RxReason_t reason);
static void prv_schedule_beacon_window(void);
static void prv_schedule_reminder_window(void);
static void prv_schedule_sensor_and_tx(void);

static uint32_t prv_get_tx_base_offset_sec(void)
{
    /* 최신 요구:
     * - 1M / 2M / normal 모두 ND 데이터 송신 시작은 +30초 기준
     * - 노드 번호별 2초 slot만 다르고, 같은 cycle 안에서는 같은 hop 주파수를 사용한다. */
    return 30u;
}

static uint32_t prv_periodic_slot_id_from_epoch_sec(uint32_t epoch_sec, uint32_t period_sec, uint32_t offset_sec)
{
    if (period_sec == 0u)
    {
        return 0xFFFFFFFFu;
    }

    if (epoch_sec < offset_sec)
    {
        return 0u;
    }

    return ((epoch_sec - offset_sec) / period_sec);
}

static uint32_t prv_floor_cycle_anchor_sec(uint32_t epoch_sec, uint32_t period_sec)
{
    if (period_sec == 0u)
    {
        return epoch_sec;
    }

    if (s_last_beacon_anchor_sec != 0u)
    {
        if (epoch_sec >= s_last_beacon_anchor_sec)
        {
            return s_last_beacon_anchor_sec +
                   (((epoch_sec - s_last_beacon_anchor_sec) / period_sec) * period_sec);
        }

        return s_last_beacon_anchor_sec;
    }

    return ((epoch_sec / period_sec) * period_sec);
}

static uint32_t prv_get_data_freq_anchor_sec(uint32_t now_sec)
{
    uint32_t period_sec = s_test_mode ? 60u : prv_get_normal_cycle_sec();
    uint32_t cycle_anchor_sec = prv_floor_cycle_anchor_sec(now_sec, period_sec);

    /* 같은 beacon cycle에서는 ND ID/slot과 무관하게 같은 hop anchor를 사용한다.
     * slot 차이는 송신 시각만 다르고 주파수는 cycle 공통으로 고정한다. */
    return cycle_anchor_sec + prv_get_tx_base_offset_sec();
}

static bool prv_get_next_predicted_beacon_centi(uint64_t now_centi, uint64_t* out_next_centi)
{
    uint32_t interval_sec;
    uint64_t anchor_centi;
    uint64_t step_centi;
    uint64_t elapsed;
    uint64_t steps;

    if (out_next_centi == NULL)
    {
        return false;
    }

    interval_sec = prv_get_beacon_interval_sec();
    if ((interval_sec == 0u) || (s_last_beacon_anchor_sec == 0u))
    {
        return false;
    }

    anchor_centi = (uint64_t)s_last_beacon_anchor_sec * 100u;
    step_centi = (uint64_t)interval_sec * 100u;

    if (now_centi < anchor_centi)
    {
        *out_next_centi = anchor_centi;
        return true;
    }

    elapsed = now_centi - anchor_centi;
    steps = (elapsed / step_centi) + 1u;
    *out_next_centi = anchor_centi + (steps * step_centi);
    return true;
}

static uint32_t prv_get_beacon_early_wake_ms(void)
{
    uint32_t base = prv_is_two_minute_mode_active() ? ND_BEACON_EARLY_WAKE_MS_2M : ND_BEACON_EARLY_WAKE_MS;
    uint32_t max  = prv_is_two_minute_mode_active() ? ND_BEACON_EARLY_WAKE_MAX_MS_2M : ND_BEACON_EARLY_WAKE_MAX_MS;
    uint32_t extra = (uint32_t)s_beacon_miss_count * ND_BEACON_EARLY_WAKE_STEP_MS;
    uint32_t early = base + extra;

    if (early > max)
    {
        early = max;
    }

    return early;
}

static uint32_t prv_get_beacon_window_ms(void)
{
    uint32_t early = prv_get_beacon_early_wake_ms();
    uint32_t tail  = prv_is_two_minute_mode_active() ? ND_BEACON_TAIL_MS_2M : ND_BEACON_TAIL_MS_NORMAL;
    return early + tail;
}

static bool prv_select_next_gw_phase_scan(void)
{
    if (s_boot_listen_active)
    {
        return true;
    }

    if (s_sync_state == ND_SYNC_UNSYNC_SEARCH)
    {
        return true;
    }

    s_gw_phase_scan_cycle_count++;
    if (s_gw_phase_scan_cycle_count >= ND_GW_PHASE_SCAN_EVERY_N_CYCLES)
    {
        s_gw_phase_scan_cycle_count = 0u;
        return true;
    }

    return false;
}

static uint32_t prv_get_beacon_rx_window_ms_with_phase_scan(void)
{
    uint32_t window_ms = prv_get_beacon_window_ms();

    if (s_force_gw_phase_scan)
    {
        window_ms += ND_GW_PHASE_SCAN_EXTRA_MS;
    }

    return window_ms;
}

static uint32_t prv_get_unsync_search_delay_ms(void)
{
    uint32_t now_ms = UTIL_TIMER_GetCurrentTime();
    return ND_SEARCH_SCAN_INTERVAL_MS_BASE + (now_ms % ND_SEARCH_SCAN_INTERVAL_MS_JITTER);
}

static uint32_t prv_pack_sync_meta(const uint8_t setting_ascii[3])
{
    return ((uint32_t)setting_ascii[0])
         | ((uint32_t)setting_ascii[1] << 8u)
         | ((uint32_t)setting_ascii[2] << 16u);
}

static void prv_unpack_sync_meta(uint32_t meta, uint8_t setting_ascii[3])
{
    setting_ascii[0] = (uint8_t)(meta & 0xFFu);
    setting_ascii[1] = (uint8_t)((meta >> 8u) & 0xFFu);
    setting_ascii[2] = (uint8_t)((meta >> 16u) & 0xFFu);
}

static bool prv_setting_ascii_is_valid(const uint8_t setting_ascii[3])
{
    uint8_t d0 = setting_ascii[0];
    uint8_t d1 = setting_ascii[1];
    char unit = (char)setting_ascii[2];

    return (d0 >= (uint8_t)'0') && (d0 <= (uint8_t)'9') &&
           (d1 >= (uint8_t)'0') && (d1 <= (uint8_t)'9') &&
           ((unit == 'M') || (unit == 'H'));
}

static uint16_t prv_calc_sync_anchor_crc(uint32_t epoch_sec, uint32_t meta, const uint8_t net_id[UI_NET_ID_LEN])
{
    uint8_t buf[4u + 3u + UI_NET_ID_LEN];

    buf[0] = (uint8_t)(epoch_sec & 0xFFu);
    buf[1] = (uint8_t)((epoch_sec >> 8u) & 0xFFu);
    buf[2] = (uint8_t)((epoch_sec >> 16u) & 0xFFu);
    buf[3] = (uint8_t)((epoch_sec >> 24u) & 0xFFu);
    buf[4] = (uint8_t)(meta & 0xFFu);
    buf[5] = (uint8_t)((meta >> 8u) & 0xFFu);
    buf[6] = (uint8_t)((meta >> 16u) & 0xFFu);
    memcpy(&buf[7], net_id, UI_NET_ID_LEN);

    return UI_CRC16_CCITT(buf, sizeof(buf), UI_CRC16_INIT);
}

static void prv_save_sync_anchor_from_beacon(const UI_Beacon_t* beacon)
{
    uint32_t epoch_sec;
    uint32_t meta;
    uint16_t crc;
    const UI_Config_t* cfg;

    if (beacon == NULL)
    {
        return;
    }

    cfg = UI_GetConfig();
    epoch_sec = UI_Time_Epoch2016_FromCalendar(&beacon->dt);
    meta = prv_pack_sync_meta(beacon->setting_ascii);
    crc = prv_calc_sync_anchor_crc(epoch_sec, meta, cfg->net_id);

    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, ND_SYNC_BKP_DR_MAGIC, ND_SYNC_BKP_MAGIC);
    HAL_RTCEx_BKUPWrite(&hrtc, ND_SYNC_BKP_DR_EPOCH, epoch_sec);
    HAL_RTCEx_BKUPWrite(&hrtc, ND_SYNC_BKP_DR_META, meta);
    HAL_RTCEx_BKUPWrite(&hrtc, ND_SYNC_BKP_DR_CRC, (uint32_t)crc);

    s_last_beacon_anchor_sec = epoch_sec;
}

static bool prv_load_sync_anchor(uint32_t* out_epoch_sec, uint8_t out_setting_ascii[3])
{
    uint32_t magic;
    uint32_t epoch_sec;
    uint32_t meta;
    uint16_t crc_stored;
    uint16_t crc_expected;
    const UI_Config_t* cfg = UI_GetConfig();

    if ((!UI_Time_IsValid()) || (out_setting_ascii == NULL))
    {
        return false;
    }

    magic = HAL_RTCEx_BKUPRead(&hrtc, ND_SYNC_BKP_DR_MAGIC);
    if (magic != ND_SYNC_BKP_MAGIC)
    {
        return false;
    }

    epoch_sec = HAL_RTCEx_BKUPRead(&hrtc, ND_SYNC_BKP_DR_EPOCH);
    meta = HAL_RTCEx_BKUPRead(&hrtc, ND_SYNC_BKP_DR_META);
    crc_stored = (uint16_t)(HAL_RTCEx_BKUPRead(&hrtc, ND_SYNC_BKP_DR_CRC) & 0xFFFFu);

    prv_unpack_sync_meta(meta, out_setting_ascii);
    if (!prv_setting_ascii_is_valid(out_setting_ascii))
    {
        return false;
    }

    crc_expected = prv_calc_sync_anchor_crc(epoch_sec, meta, cfg->net_id);
    if (crc_expected != crc_stored)
    {
        return false;
    }

    if (out_epoch_sec != NULL)
    {
        *out_epoch_sec = epoch_sec;
    }

    return true;
}

static void prv_enter_locked_from_beacon(const UI_Beacon_t* beacon)
{
    prv_save_sync_anchor_from_beacon(beacon);
    s_sync_state = ND_SYNC_LOCKED;
    s_beacon_ok = true;
    s_beacon_miss_count = 0u;
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    s_runtime_enabled = true;
    s_sensor_ready = false;
}

static bool prv_try_enter_holdover_from_backup(void)
{
    uint8_t setting_ascii[3];
    uint32_t epoch_sec = 0u;

    if (!prv_load_sync_anchor(&epoch_sec, setting_ascii))
    {
        return false;
    }

    prv_apply_setting_ascii(setting_ascii);
    s_last_beacon_anchor_sec = epoch_sec;
    s_sync_state = ND_SYNC_HOLDOVER;
    s_beacon_ok = false;
    s_runtime_enabled = true;
    s_sensor_ready = false;
    if (s_beacon_miss_count < 1u)
    {
        s_beacon_miss_count = 1u;
    }
    return true;
}

static void prv_stop_sensor_and_tx_timers(void)
{
    (void)UTIL_TIMER_Stop(&s_tmr_sensor_sched);
    (void)UTIL_TIMER_Stop(&s_tmr_tx_sched);
    (void)UTIL_TIMER_Stop(&s_tmr_reminder_sched);
}

static void prv_schedule_unsync_search_scan(void)
{
    uint32_t delay_ms = prv_get_unsync_search_delay_ms();

    (void)UTIL_TIMER_Stop(&s_tmr_beacon_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_beacon_sched, delay_ms);
    (void)UTIL_TIMER_Start(&s_tmr_beacon_sched);
}

static void prv_enter_unsync_search(void)
{
    s_sync_state = ND_SYNC_UNSYNC_SEARCH;
    s_beacon_ok = false;
    s_runtime_enabled = UI_Time_IsValid();
    s_sensor_ready = false;
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    s_force_gw_phase_scan = true;
    s_gw_phase_scan_cycle_count = 0u;

    if (s_beacon_miss_count < 1u)
    {
        s_beacon_miss_count = 1u;
    }

    (void)UTIL_TIMER_Stop(&s_tmr_reminder_sched);

    if (s_runtime_enabled)
    {
        prv_schedule_sensor_and_tx();
    }
    else
    {
        prv_stop_sensor_and_tx_timers();
    }

    prv_schedule_unsync_search_scan();
}

static void prv_note_main_beacon_miss(void)
{
    s_beacon_ok = false;

    if (s_boot_listen_active)
    {
        return;
    }

    if ((s_sync_state == ND_SYNC_LOCKED) || (s_sync_state == ND_SYNC_HOLDOVER))
    {
        if (s_beacon_miss_count < 10u)
        {
            s_beacon_miss_count++;
        }

        if (s_beacon_miss_count > ND_SYNC_HOLDOVER_MAX_MISS)
        {
            prv_enter_unsync_search();
        }
        else
        {
            s_sync_state = ND_SYNC_HOLDOVER;
        }
    }
}

static uint32_t prv_ms_until(uint32_t deadline_ms)
{
    uint32_t now = UTIL_TIMER_GetCurrentTime();
    return ((int32_t)(deadline_ms - now) > 0) ? (deadline_ms - now) : 0u;
}

static void prv_begin_boot_listen(void)
{
    /* 전원 ON 후에는 항상 6분 full listen을 먼저 수행한다.
     * RTC/backup time가 살아 있어도, 이 구간에서는 beacon을 한 번 직접 받는 것을 우선한다. */
    s_sync_state = ND_SYNC_BOOT_SEARCH;
    s_runtime_enabled = false;
    s_sensor_ready = false;
    s_beacon_ok = false;
    s_boot_listen_active = true;
    s_boot_listen_deadline_ms = UTIL_TIMER_GetCurrentTime() + UI_ND_BOOT_LISTEN_MS;
    prv_stop_sensor_and_tx_timers();

    uint32_t first_ms = UI_ND_BOOT_LISTEN_MS;
    if (first_ms > ND_BOOT_RX_WINDOW_MS)
    {
        first_ms = ND_BOOT_RX_WINDOW_MS;
    }
    if (first_ms == 0u)
    {
        first_ms = 1u;
    }

    prv_start_beacon_rx(first_ms, ND_RX_REASON_BOOT);
}

static bool prv_restart_current_rx_window(void)
{
    ND_RxReason_t reason = s_rx_reason;
    uint32_t remain_ms = prv_ms_until(s_rx_window_deadline_ms);

    if (remain_ms < 20u)
    {
        return false;
    }

    if (reason == ND_RX_REASON_NONE)
    {
        if (s_boot_listen_active)
        {
            reason = ND_RX_REASON_BOOT;
        }
        else if (s_sync_state == ND_SYNC_UNSYNC_SEARCH)
        {
            reason = ND_RX_REASON_SEARCH;
        }
        else
        {
            reason = ND_RX_REASON_MAIN;
        }
    }

    prv_start_beacon_rx(remain_ms, reason);
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

            prv_start_beacon_rx(next_ms, ND_RX_REASON_BOOT);
            return;
        }

        s_boot_listen_active = false;
        s_boot_listen_deadline_ms = 0u;

        if (!prv_try_enter_holdover_from_backup())
        {
            prv_enter_unsync_search();
            return;
        }
    }

    prv_schedule_beacon_window();
    if (s_runtime_enabled)
    {
        prv_schedule_reminder_window();
        prv_schedule_sensor_and_tx();
    }
    else
    {
        prv_stop_sensor_and_tx_timers();
    }
}

static void prv_start_beacon_rx(uint32_t window_ms, ND_RxReason_t reason)
{
    /* radio 사용 중이면 무시 */
    if (s_state != ND_STATE_IDLE) return;

    if (window_ms == 0u)
    {
        window_ms = 1u;
    }

    if (!prv_radio_ready_for_rx()) return;
    if (!UI_Radio_PrepareRx(UI_BEACON_PAYLOAD_LEN))
    {
        return;
    }

    /* 주 비콘 window 시작 시에만 최근 beacon 상태를 다시 판정한다. */
    if (reason != ND_RX_REASON_REMINDER)
    {
        s_beacon_ok = false;
    }
    s_rx_window_deadline_ms = UTIL_TIMER_GetCurrentTime() + window_ms;
    s_rx_reason = reason;

    UI_LPM_LockStop();
    s_state = ND_STATE_RX_BEACON;

    Radio.SetChannel(UI_RF_GetBeaconFreqHz());
    Radio.Rx(window_ms);
}

static void prv_schedule_beacon_window(void)
{
    uint64_t now;
    uint32_t interval_sec;
    uint64_t next;
    uint64_t delta_centi;
    uint32_t delta_ms;
    uint32_t early_ms;

    if (!UI_Time_IsValid())
    {
        s_force_gw_phase_scan = true;
        prv_schedule_unsync_search_scan();
        return;
    }

    now = UI_Time_NowCenti2016();
    interval_sec = prv_get_beacon_interval_sec();

    if (!prv_get_next_predicted_beacon_centi(now, &next))
    {
        if (s_sync_state == ND_SYNC_UNSYNC_SEARCH)
        {
            s_force_gw_phase_scan = true;
            prv_schedule_unsync_search_scan();
            return;
        }

        next = prv_next_event_centi(now, interval_sec, 0u);
    }

    s_force_gw_phase_scan = prv_select_next_gw_phase_scan();

    delta_centi = (next > now) ? (next - now) : 1u;
    delta_ms = (uint32_t)(delta_centi * 10u);
    early_ms = prv_get_beacon_early_wake_ms();

    if (s_force_gw_phase_scan)
    {
        early_ms += ND_GW_PHASE_SCAN_EXTRA_MS;
    }

    if (delta_ms > early_ms)
    {
        delta_ms -= early_ms;
    }
    else
    {
        delta_ms = 1u;
    }

    (void)UTIL_TIMER_Stop(&s_tmr_beacon_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_beacon_sched, delta_ms);
    (void)UTIL_TIMER_Start(&s_tmr_beacon_sched);
}

static void prv_schedule_reminder_window(void)
{
    /* 최신 요구:
     * - GW beacon은 한 번만 송신
     * - ND는 reminder listen 대신 다음 예측 beacon window에서 저전력 재동기를 수행한다. */
    (void)UTIL_TIMER_Stop(&s_tmr_reminder_sched);
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

    /* SETTING:2M은 +30초부터 node별 2초 슬롯을 사용하고, GW 요구에 맞춰 최대 10노드 제한 */
    if (prv_is_two_minute_mode_active() && node >= 10u)
    {
        return;
    }

    uint64_t now = UI_Time_NowCenti2016();

    uint32_t period = s_test_mode ? 60u : prv_get_normal_cycle_sec();
    uint32_t sensor_off = s_test_mode ? UI_ND_SENSOR_START_S_TEST : UI_ND_SENSOR_START_S_NORMAL;
    uint32_t base_tx = prv_get_tx_base_offset_sec();

    if (prv_is_two_minute_mode_active())
    {
        sensor_off = 6u;
    }

    /* TX offset:
     *  - 01M test mode: +30초부터 node별 2초 슬롯 송신
     *  - 02M mode     : +30초부터 node별 2초 슬롯 송신
     *  - normal mode  : 설정 주기 기준 +30초부터 node별 2초 슬롯 송신 */
    uint32_t tx_off = base_tx + (uint32_t)node * 2u;

    uint64_t next_sensor = prv_next_event_centi(now, period, sensor_off);
    uint64_t next_tx     = prv_next_event_centi(now, period, tx_off);

    /* 센서/송신 예약 */
    uint32_t ds_ms = (uint32_t)((next_sensor > now) ? ((next_sensor - now) * 10u) : 1u);
    uint32_t dt_ms = (uint32_t)((next_tx > now) ? ((next_tx - now) * 10u) : 1u);

    /* slot 경계 정각 전송은 타이머/재arm 지터에 약하다.
     * 전송을 slot 안쪽으로 200ms 넣어서 GW RX slot 중앙에서 수신되게 한다. */
    dt_ms += ND_TX_IN_SLOT_DELAY_MS;

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

    if ((ev & ND_EVT_BOOT_LISTEN_START) != 0u)
    {
        /* 부팅 후 6분은 6초 window를 반복해서 beacon을 잡는다. */
        prv_begin_boot_listen();
        return;
    }

    if ((ev & ND_EVT_BEACON_LISTEN_START) != 0u)
    {
        if (!UI_Time_IsValid())
        {
            s_force_gw_phase_scan = true;
            prv_start_beacon_rx(ND_SEARCH_RX_WINDOW_MS, ND_RX_REASON_SEARCH);
        }
        else if (s_sync_state == ND_SYNC_UNSYNC_SEARCH)
        {
            uint32_t window_ms = (s_last_beacon_anchor_sec != 0u)
                               ? prv_get_beacon_rx_window_ms_with_phase_scan()
                               : ND_SEARCH_RX_WINDOW_MS;
            prv_start_beacon_rx(window_ms, ND_RX_REASON_SEARCH);
        }
        else
        {
            prv_start_beacon_rx(prv_get_beacon_rx_window_ms_with_phase_scan(), ND_RX_REASON_MAIN);
        }

        s_force_gw_phase_scan = false;
        return;
    }

    if ((ev & ND_EVT_REMINDER_LISTEN_START) != 0u)
    {
        /* reminder beacon disabled */
        return;
    }

    if ((ev & ND_EVT_SENSOR_START) != 0u)
    {
        uint32_t now_sec;
        uint32_t period;
        uint32_t sensor_off;
        uint32_t sensor_slot_id;

        if (!s_runtime_enabled)
        {
            s_sensor_ready = false;
            return;
        }

        now_sec = UI_Time_NowSec2016();
        period = s_test_mode ? 60u : prv_get_normal_cycle_sec();
        sensor_off = s_test_mode ? UI_ND_SENSOR_START_S_TEST : UI_ND_SENSOR_START_S_NORMAL;
        if (prv_is_two_minute_mode_active())
        {
            sensor_off = 6u;
        }

        sensor_slot_id = prv_periodic_slot_id_from_epoch_sec(now_sec, period, sensor_off);
        if (sensor_slot_id == s_last_sensor_slot_id)
        {
            return;
        }
        s_last_sensor_slot_id = sensor_slot_id;

        s_sensor_ready = false;
        (void)ND_Sensors_MeasureAll(&s_last_sensor);
        s_sensor_ready = true;
        prv_led1_pulse_10ms();
        return;
    }

    if ((ev & ND_EVT_TX_START) != 0u)
    {
        const UI_Config_t* cfg;
        UI_NodeData_t nd;
        uint32_t now_sec;
        uint32_t period;
        uint32_t tx_off;
        uint32_t tx_slot_id;
        uint32_t freq_anchor_sec;

        if (!s_runtime_enabled)
        {
            return;
        }

        if (!s_sensor_ready)
        {
            return;
        }

        cfg = UI_GetConfig();
        now_sec = UI_Time_NowSec2016();
        period = s_test_mode ? 60u : prv_get_normal_cycle_sec();
        tx_off = prv_get_tx_base_offset_sec() + (uint32_t)cfg->node_num * 2u;
        tx_slot_id = prv_periodic_slot_id_from_epoch_sec(now_sec, period, tx_off);

        if (tx_slot_id == s_last_tx_slot_id)
        {
            return;
        }
        s_last_tx_slot_id = tx_slot_id;

        memset(&nd, 0, sizeof(nd));
        nd.node_num = cfg->node_num;
        memcpy(nd.net_id, cfg->net_id, UI_NET_ID_LEN);
        nd.batt_lvl = s_last_sensor.batt_lvl;
        nd.temp_c = s_last_sensor.temp_c;
        nd.beacon_cnt = s_beacon_cnt;
        nd.x = s_last_sensor.x;
        nd.y = s_last_sensor.y;
        nd.z = s_last_sensor.z;
        nd.adc = s_last_sensor.adc;
        nd.pulse_cnt = s_last_sensor.pulse_cnt;

        (void)UI_Pkt_BuildNodeData(s_node_tx_payload, &nd);

        if (!prv_radio_ready_for_tx())
        {
            return;
        }
        if (!UI_Radio_PrepareTx(UI_NODE_PAYLOAD_LEN))
        {
            return;
        }

        freq_anchor_sec = prv_get_data_freq_anchor_sec(now_sec);

        UI_LPM_LockStop();
        s_state = ND_STATE_TX_DATA;
        Radio.SetChannel(UI_RF_GetDataFreqHz(freq_anchor_sec, period, cfg->node_num));
        Radio.Send(s_node_tx_payload, UI_NODE_PAYLOAD_LEN);
        return;
    }
}

void ND_App_Init(void)
{
    if (s_inited)
    {
        return;
    }

    prv_refresh_mode_from_config();
    prv_power_on_led_blink();

    UTIL_TIMER_Create(&s_tmr_boot_listen, 0u, UTIL_TIMER_ONESHOT, prv_tmr_boot_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_beacon_sched, 0u, UTIL_TIMER_ONESHOT, prv_tmr_beacon_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_reminder_sched, 0u, UTIL_TIMER_ONESHOT, prv_tmr_reminder_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_sensor_sched, 0u, UTIL_TIMER_ONESHOT, prv_tmr_sensor_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_tx_sched, 0u, UTIL_TIMER_ONESHOT, prv_tmr_tx_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_led1_pulse, 10u, UTIL_TIMER_ONESHOT, prv_led1_pulse_off_cb, NULL);

    s_state = ND_STATE_IDLE;
    s_beacon_ok = false;
    s_beacon_cnt = 0u;
    s_runtime_enabled = false;
    s_beacon_miss_count = 0u;
    s_last_beacon_anchor_sec = 0u;
    s_sensor_ready = false;
    s_last_sensor_slot_id = 0xFFFFFFFFu;
    s_last_tx_slot_id = 0xFFFFFFFFu;
    s_force_gw_phase_scan = false;
    s_gw_phase_scan_cycle_count = 0u;

    s_inited = true;
    s_evt_flags |= ND_EVT_BOOT_LISTEN_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

void ND_Radio_OnTxDone(void)
{
    Radio.Sleep();
    s_state = ND_STATE_IDLE;
    UI_LPM_UnlockStop();
}

void ND_Radio_OnTxTimeout(void)
{
    Radio.Sleep();
    s_state = ND_STATE_IDLE;
    UI_LPM_UnlockStop();
}

void ND_Radio_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    UI_Beacon_t beacon;
    const UI_Config_t* cfg = UI_GetConfig();
    (void)rssi;
    (void)snr;

    if (s_state != ND_STATE_RX_BEACON)
    {
        return;
    }

    Radio.Sleep();
    s_state = ND_STATE_IDLE;
    UI_LPM_UnlockStop();

    if (!UI_Pkt_ParseBeacon(payload, size, &beacon))
    {
        if (prv_restart_current_rx_window())
        {
            return;
        }

        if (s_rx_reason == ND_RX_REASON_MAIN)
        {
            prv_note_main_beacon_miss();
        }
        prv_continue_boot_listen_or_schedule();
        return;
    }

    if (memcmp(beacon.net_id, cfg->net_id, UI_NET_ID_LEN) != 0)
    {
        if (prv_restart_current_rx_window())
        {
            return;
        }

        if (s_rx_reason == ND_RX_REASON_MAIN)
        {
            prv_note_main_beacon_miss();
        }
        prv_continue_boot_listen_or_schedule();
        return;
    }

    /* NOTE: original project time-set API mismatch caused link error.
     * Keep beacon anchor update and setting sync; actual RTC set should use the project's existing time API. */
    (void)ND_BEACON_RX_TIME_CORR_CENTI;
    prv_apply_setting_ascii(beacon.setting_ascii);

    s_beacon_cnt++;
    prv_enter_locked_from_beacon(&beacon);
    prv_continue_boot_listen_or_schedule();
}

void ND_Radio_OnRxTimeout(void)
{
    if (s_state != ND_STATE_RX_BEACON)
    {
        return;
    }

    Radio.Sleep();
    s_state = ND_STATE_IDLE;
    UI_LPM_UnlockStop();

    if (s_rx_reason == ND_RX_REASON_MAIN)
    {
        prv_note_main_beacon_miss();
    }
    else if (s_rx_reason == ND_RX_REASON_SEARCH)
    {
        s_beacon_ok = false;
    }

    prv_continue_boot_listen_or_schedule();
}

void ND_Radio_OnRxError(void)
{
    if (s_state != ND_STATE_RX_BEACON)
    {
        return;
    }

    Radio.Sleep();
    s_state = ND_STATE_IDLE;
    UI_LPM_UnlockStop();

    if (prv_restart_current_rx_window())
    {
        return;
    }

    if (s_rx_reason == ND_RX_REASON_MAIN)
    {
        prv_note_main_beacon_miss();
    }

    prv_continue_boot_listen_or_schedule();
}
