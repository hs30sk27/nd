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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

extern RTC_HandleTypeDef hrtc;

/* -------------------------------------------------------------------------- */
/* Node 상태 */
/* -------------------------------------------------------------------------- */
typedef enum {
    ND_STATE_IDLE = 0,
    ND_STATE_RX_BEACON,
    ND_STATE_TX_DATA,
} ND_State_t;

typedef enum {
    ND_RX_REASON_NONE = 0,
    ND_RX_REASON_BOOT,
    ND_RX_REASON_MAIN,
    ND_RX_REASON_REMINDER,
    ND_RX_REASON_SEARCH,
    ND_RX_REASON_SYNC,
} ND_RxReason_t;

typedef enum {
    ND_SYNC_BOOT_SEARCH = 0,
    ND_SYNC_LOCKED,
    ND_SYNC_HOLDOVER,
    ND_SYNC_PHASE_WALK,
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
static UTIL_TIMER_Object_t s_tmr_tx_watchdog;
static UTIL_TIMER_Object_t s_tmr_led1_pulse;
static UTIL_TIMER_Object_t s_tmr_boot_led_blink;
static UTIL_TIMER_Object_t s_tmr_test_session;

static volatile uint32_t s_evt_flags = 0;
#define ND_EVT_BOOT_LISTEN_START      (1u << 0)
#define ND_EVT_BEACON_LISTEN_START    (1u << 1)
#define ND_EVT_SENSOR_START           (1u << 2)
#define ND_EVT_TX_START               (1u << 3)
#define ND_EVT_REMINDER_LISTEN_START  (1u << 4)
#define ND_EVT_TX_RECOVER             (1u << 5)
#define ND_EVT_TEST_SESSION_EXPIRE     (1u << 6)
#define ND_EVT_SYNC_START              (1u << 7)
#define ND_EVT_ENTER_STOP              (1u << 8)
#define ND_EVT_SYNC_TX_NOTIFY          (1u << 9)
#define ND_EVT_SYNC_DONE_NOTIFY        (1u << 10)
#define ND_EVT_SYNC_TIMEOUT_NOTIFY     (1u << 11)
#define ND_EVT_SYNC_TX_FAIL_NOTIFY     (1u << 12)
#define ND_EVT_RADIO_TX_DONE_LED_PULSE (1u << 13)
#define ND_EVT_RADIO_RX_DONE_LED_PULSE (1u << 14)
#define ND_EVT_SYNC_NOTIFY_MASK        (ND_EVT_SYNC_TX_NOTIFY | ND_EVT_SYNC_DONE_NOTIFY | ND_EVT_SYNC_TIMEOUT_NOTIFY | ND_EVT_SYNC_TX_FAIL_NOTIFY)

static bool s_beacon_ok = false;
static uint16_t s_beacon_cnt = 0;
static bool s_test_mode = false;
static ND_SyncState_t s_sync_state = ND_SYNC_BOOT_SEARCH;
static bool s_runtime_enabled = false;
static uint8_t s_beacon_miss_count = 0u;
static uint8_t s_phase_walk_idx = 0u;
static uint32_t s_last_beacon_anchor_sec = 0u;
static ND_SensorResult_t s_last_sensor;
static bool s_sensor_ready = false;
static bool s_last_sensor_valid = false;
static uint8_t s_node_tx_payload[UI_NODE_PAYLOAD_LEN];
static uint8_t s_node_tx_payload_len = 0u;
static uint8_t s_sync_req_tx_payload[UI_NODE_PAYLOAD_LEN];
static uint8_t s_sync_req_tx_payload_len = 0u;
static bool s_sync_cmd_active = false;
static bool s_sync_tx_wait_beacon_pending = false;
static bool s_sync_ble_timeout_saved_valid = false;
static uint32_t s_sync_ble_timeout_saved_ms = 0u;
static bool s_ble_runtime_resume_allowed = false;
static uint8_t s_unsync_backoff_idx = 0u;
static uint32_t s_last_sensor_slot_id = 0xFFFFFFFFu;
static uint32_t s_last_tx_slot_id = 0xFFFFFFFFu;
static uint32_t s_tx_inflight_slot_id = 0xFFFFFFFFu;
static bool s_test_session_active = false;
static uint8_t s_test_session_restore_value = 0u;
static char s_test_session_restore_unit = 'H';

/* Unsync search backoff is expressed as beacon-slot counts.
 * Default 5-minute beacon period => 12/24/48/72/144/144 slots
 * maps to 1h/2h/4h/6h/12h/12h, and after reaching 144 it keeps
 * repeating 144-beacon intervals indefinitely. Any valid beacon
 * reception resets the index back to the first step. */
static const uint16_t k_unsync_backoff_beacon_count[] = {
    12u,
    24u,
    48u,
    72u,
    144u,
    144u,
};

#define ND_TX_IN_SLOT_DELAY_MS            (40u)
#define ND_TX_SLOT0_EXTRA_DELAY_MS        (0u)
#define ND_TX_DUE_LATE_GRACE_CENTI        ((UI_SLOT_DURATION_MS / 10u) - 10u)
#define ND_TX_RETRY_DELAY_MS              (20u)
#define ND_TX_RETRY_GUARD_MS              (20u)
#define ND_TX_WATCHDOG_MS                (4000u)
#define ND_BOOT_RX_WINDOW_MS              (6000u)
#define ND_BOOT_BEACON_ON_MS              (20u)
#define ND_BOOT_BEACON_OFF_MS             (480u)
#define ND_RADIO_LED_PULSE_MS            (50u)
#define ND_BEACON_EARLY_WAKE_MS           (1000u)
#define ND_BEACON_EARLY_WAKE_MS_2M        (500u)
#define ND_BEACON_EARLY_WAKE_STEP_MS      (500u)
#define ND_BEACON_EARLY_WAKE_MAX_MS       (2500u)
#define ND_BEACON_EARLY_WAKE_MAX_MS_2M    (1500u)
#define ND_BEACON_TAIL_MS_NORMAL          (5600u)
#define ND_BEACON_TAIL_MS_2M              (5800u)
#define ND_BEACON_RX_TIME_CORR_CENTI      (74u)
#define ND_GW_PHASE_SCAN_EVERY_N_CYCLES   (12u)
#define ND_GW_PHASE_SCAN_EXTRA_MS         (4500u)
#define ND_REMINDER_RX_WINDOW_MS_BASE     (2200u)
#define ND_REMINDER_RX_WINDOW_MS_MAX      (3600u)
#define ND_REMINDER_RX_WINDOW_STEP_MS     (200u)
#define ND_REMINDER_TX_GUARD_MS           (600u)
#define ND_SYNC_HOLDOVER_MAX_MISS         (8u)
#define ND_SYNC_PHASE_WALK_MAX_MISS       (24u)
#define ND_PHASE_WALK_RX_WINDOW_MS        (1500u)
#define ND_SEARCH_RX_WINDOW_MS            (2000u)
#define ND_SYNC_CMD_RX_WINDOW_MS          (5000u)
#define ND_SYNC_CMD_BLE_HOLD_MS           (15000u)
#define ND_SYNC_NOTIFY_TX_STR             ""
#define ND_SYNC_NOTIFY_DONE_STR           "OK\r\n"
#define ND_SYNC_NOTIFY_TIMEOUT_STR        "FAIL\r\n"
#define ND_SYNC_NOTIFY_TX_FAIL_STR        "FAIL\r\n"
#define ND_SEARCH_SCAN_INTERVAL_MS_BASE   (90000u)
#define ND_SEARCH_SCAN_INTERVAL_MS_JITTER (30000u)
#define ND_SYNC_BKP_MAGIC                 (0x4E445359u)
/*
 * RTC backup register usage separation
 * - timer_if.c : DR0~DR2
 * - ui_time.c  : DR4~DR7
 * - nd sync    : DR8~DR11
 */
#define ND_SYNC_BKP_DR_MAGIC              (RTC_BKP_DR8)
#define ND_SYNC_BKP_DR_EPOCH              (RTC_BKP_DR9)
#define ND_SYNC_BKP_DR_META               (RTC_BKP_DR10)
#define ND_SYNC_BKP_DR_CRC                (RTC_BKP_DR11)
#define ND_RX_RETRY_DELAY_MS              (100u)
#define ND_TEST_SESSION_MS             (60u * 60u * 1000u)

#ifndef ND_INTERNAL_TEMP_COMP_C
#define ND_INTERNAL_TEMP_COMP_C ((int8_t)0)
#endif

static int8_t prv_apply_nd_internal_temp_comp(int8_t temp_c)
{
    int16_t v;

    if (temp_c == UI_NODE_TEMP_INVALID_C) {
        return UI_NODE_TEMP_INVALID_C;
    }

    v = (int16_t)temp_c + (int16_t)ND_INTERNAL_TEMP_COMP_C;
    if (v < (int16_t)UI_NODE_TEMP_MIN_C) {
        v = (int16_t)UI_NODE_TEMP_MIN_C;
    }
    if (v > (int16_t)UI_NODE_TEMP_MAX_C) {
        v = (int16_t)UI_NODE_TEMP_MAX_C;
    }
    return (int8_t)v;
}

static uint16_t prv_scale_signed_axis_to_u16(int16_t raw)
{
    int32_t delta = (int32_t)raw;
    uint32_t numer;

    if (delta < -(int32_t)UI_NODE_AXIS_RAW_HALF_RANGE) {
        delta = -(int32_t)UI_NODE_AXIS_RAW_HALF_RANGE;
    }
    if (delta > (int32_t)UI_NODE_AXIS_RAW_HALF_RANGE) {
        delta = (int32_t)UI_NODE_AXIS_RAW_HALF_RANGE;
    }

    numer = (uint32_t)(delta + (int32_t)UI_NODE_AXIS_RAW_HALF_RANGE) * UI_NODE_AXIS_VALID_SPAN_U16;
    numer += (uint32_t)UI_NODE_AXIS_RAW_HALF_RANGE;
    return (uint16_t)(UI_NODE_AXIS_VALID_MIN_U16 +
                      (uint16_t)(numer / (uint32_t)(2u * UI_NODE_AXIS_RAW_HALF_RANGE)));
}

static uint16_t prv_scale_adc_avg_to_u16(uint16_t raw)
{
    uint32_t numer = ((uint32_t)raw * UI_NODE_ADC_SCALED_MAX_U16) + (UI_NODE_ADC_RAW_MAX / 2u);
    return (uint16_t)(numer / UI_NODE_ADC_RAW_MAX);
}

static uint16_t prv_encode_payload_x(int16_t raw)
{
    return prv_scale_signed_axis_to_u16(raw);
}

static uint16_t prv_encode_payload_y(int16_t raw)
{
    return prv_scale_signed_axis_to_u16(raw);
}

static uint16_t prv_encode_payload_z(int16_t raw)
{
    return prv_scale_signed_axis_to_u16(raw);
}

static uint16_t prv_encode_payload_adc(uint16_t raw)
{
    return prv_scale_adc_avg_to_u16(raw);
}

static void prv_stop_tx_watchdog(void);
static void prv_refresh_runtime_timers_after_tx(void);
static void prv_force_tx_recovery(bool keep_last_slot);
static void prv_stop_sensor_and_tx_timers(void);
static void prv_schedule_beacon_window(void);
static void prv_schedule_reminder_window(void);
static void prv_schedule_sensor_and_tx(void);
static void prv_continue_boot_listen_or_schedule(void);
static void prv_enter_unsync_search(void);
static uint32_t prv_get_unsync_backoff_sec(void);
static void prv_reset_unsync_backoff(void);
static void prv_advance_unsync_backoff(void);
static uint32_t prv_get_runtime_cycle_sec(void);
static void prv_refresh_mode_from_config(void);
static bool prv_format_setting_ascii(uint8_t value, char unit, uint8_t out_setting_ascii[3]);
static void prv_request_immediate_beacon_scan(void);
static void prv_start_test_session_from_cmd(void);
static void prv_stop_test_session(void);
static void prv_clear_pending_runtime_rx_events(void);
static bool prv_abort_beacon_rx_for_tx(void);
static void prv_suspend_runtime_rx_around_tx(void);
static void prv_start_tx_watchdog(void);
static void prv_enter_stop_now_if_possible(void);
static void prv_request_stop_mode(void);
static void prv_request_stop_mode_if_possible(void);
static void prv_enter_unsynced_idle(void);
static void prv_resume_schedule_after_boot_search(void);
static void prv_abort_active_radio_for_ble(void);
static bool prv_try_enter_holdover_from_backup(void);
static void prv_begin_boot_listen(void);
static bool prv_should_pause_runtime_for_ble(void);
static void prv_resume_runtime_after_sync_completion(void);

static bool s_boot_listen_active = false;
static bool s_boot_listen_stop_after_ble = false;
static uint32_t s_boot_listen_deadline_ms = 0u;
static bool s_boot_led_blink_active = false;
static bool s_boot_led_blink_on = false;
static uint32_t s_rx_window_deadline_ms = 0u;
static ND_RxReason_t s_rx_reason = ND_RX_REASON_NONE;
static bool s_force_gw_phase_scan = false;
static uint8_t s_gw_phase_scan_cycle_count = 0u;

static bool prv_radio_ready_for_tx(void)
{
    return (Radio.SetChannel != NULL) && (Radio.Send != NULL) && (Radio.Sleep != NULL);
}

static bool prv_radio_ready_for_rx(void)
{
    return (Radio.SetChannel != NULL) && (Radio.Rx != NULL) && (Radio.Sleep != NULL);
}

static void prv_hold_ble_for_sync(void)
{
    /*
     * BLE 세션 유지/연장은 버튼에 의해서만 이뤄진다.
     * SYNC 상태 메시지는 이미 켜져 있는 BLE 세션에서만 전송하고,
     * 여기서는 UART 준비만 보장한다.
     */
    if (!UI_BLE_IsActive()) {
        return;
    }

    UI_BLE_EnsureSerialReady();
}

static void prv_clear_saved_ble_timeout_for_sync(void)
{
    s_sync_ble_timeout_saved_valid = false;
    s_sync_ble_timeout_saved_ms = 0u;
}

static void prv_hold_ble_timeout_for_sync(void)
{
    uint32_t remain_ms = 0u;
    uint32_t hold_ms = ND_SYNC_CMD_BLE_HOLD_MS;

    prv_clear_saved_ble_timeout_for_sync();

    if (!UI_BLE_IsActive()) {
        return;
    }

    if (UI_BLE_GetRemainingMs(&remain_ms) && (remain_ms > 0u)) {
        s_sync_ble_timeout_saved_valid = true;
        s_sync_ble_timeout_saved_ms = remain_ms;
        if (remain_ms > hold_ms) {
            hold_ms = remain_ms;
        }
    }

    UI_BLE_ExtendMs(hold_ms);
}

static void prv_restore_ble_timeout_after_sync_success(void)
{
    if (UI_BLE_IsActive() &&
        s_sync_ble_timeout_saved_valid &&
        (s_sync_ble_timeout_saved_ms > 0u)) {
        UI_BLE_ExtendMs(s_sync_ble_timeout_saved_ms);
    }

    prv_clear_saved_ble_timeout_for_sync();
}

static bool prv_should_pause_runtime_for_ble(void)
{
    return UI_BLE_IsActive() && !s_ble_runtime_resume_allowed;
}

static void prv_resume_runtime_after_sync_completion(void)
{
    s_boot_listen_stop_after_ble = false;

    if (!UI_BLE_IsActive()) {
        s_ble_runtime_resume_allowed = false;
        return;
    }

    s_ble_runtime_resume_allowed = true;

    if (s_state != ND_STATE_IDLE) {
        return;
    }

    if (s_sync_state == ND_SYNC_UNSYNC_SEARCH) {
        prv_enter_unsync_search();
        return;
    }

    if (s_sync_state == ND_SYNC_BOOT_SEARCH) {
        if (UI_Time_IsValid()) {
            prv_resume_schedule_after_boot_search();
        } else {
            prv_begin_boot_listen();
        }
        return;
    }

    prv_continue_boot_listen_or_schedule();
}

static void prv_tmr_boot_led_blink_cb(void *context);
static void prv_boot_led_blink_start(void);
static void prv_boot_led_blink_stop(void);

static void prv_send_sync_status(const char *msg)
{
    if ((msg == NULL) || (*msg == '\0')) {
        return;
    }
    if (!UI_BLE_IsActive()) {
        return;
    }

    prv_hold_ble_for_sync();
    UI_UART_SendString(msg);
}

static void prv_led0(bool on)
{
#if defined(LED0_GPIO_Port) && defined(LED0_Pin)
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}

static void prv_led1(bool on)
{
    if (UI_BLE_IsActive()) {
        return;
    }
#if UI_HAVE_LED1
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}

static void prv_boot_beacon_leds(bool on)
{
    (void)on;

#if defined(LED0_GPIO_Port) && defined(LED0_Pin)
    if (!UI_BLE_IsActive()) {
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
#endif

#if UI_HAVE_LED1
    if (on) {
        if (!UI_BLE_IsActive()) {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        }
    } else {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }
#endif
}

static void prv_tmr_boot_led_blink_cb(void *context)
{
    uint32_t next_ms;

    (void)context;

    if (!s_boot_led_blink_active) {
        return;
    }

    s_boot_led_blink_on = !s_boot_led_blink_on;
    prv_boot_beacon_leds(s_boot_led_blink_on);
    next_ms = s_boot_led_blink_on ? ND_BOOT_BEACON_ON_MS : ND_BOOT_BEACON_OFF_MS;
    (void)UTIL_TIMER_Stop(&s_tmr_boot_led_blink);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_boot_led_blink, next_ms);
    (void)UTIL_TIMER_Start(&s_tmr_boot_led_blink);
}

static void prv_boot_led_blink_start(void)
{
    s_boot_led_blink_active = true;
    s_boot_led_blink_on = true;
    prv_boot_beacon_leds(true);
    (void)UTIL_TIMER_Stop(&s_tmr_boot_led_blink);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_boot_led_blink, ND_BOOT_BEACON_ON_MS);
    (void)UTIL_TIMER_Start(&s_tmr_boot_led_blink);
}

static void prv_boot_led_blink_stop(void)
{
    s_boot_led_blink_active = false;
    s_boot_led_blink_on = false;
    (void)UTIL_TIMER_Stop(&s_tmr_boot_led_blink);
    prv_boot_beacon_leds(false);
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

static void prv_led1_blocking_pulse_ms(uint32_t pulse_ms)
{
    if (pulse_ms == 0u) {
        pulse_ms = 1u;
    }

    prv_led1(true);
    HAL_Delay(pulse_ms);
    prv_led1(false);
}

static const char* prv_batt_text_from_level(uint8_t batt_lvl)
{
    if (batt_lvl == UI_NODE_BATT_LVL_INVALID) {
        return "-";
    }
    return (batt_lvl == UI_NODE_BATT_LVL_NORMAL) ? "3.5" : "LOW";
}

static void prv_set_invalid_sensor_result(ND_SensorResult_t* r)
{
    if (r == NULL) {
        return;
    }

    memset(r, 0, sizeof(*r));
    r->batt_lvl = UI_NODE_BATT_LVL_INVALID;
    r->temp_c = UI_NODE_TEMP_INVALID_C;
    r->x = (int16_t)0xFFFFu;
    r->y = (int16_t)0xFFFFu;
    r->z = (int16_t)0xFFFFu;
    r->adc = 0xFFFFu;
    r->pulse_cnt = 0xFFFFFFFFu;
}

static bool prv_sensor_xyz_valid(const ND_SensorResult_t* r)
{
    return (r != NULL) &&
           (r->x != (int16_t)0xFFFFu) &&
           (r->y != (int16_t)0xFFFFu) &&
           (r->z != (int16_t)0xFFFFu);
}

static bool prv_sensor_adc_valid(const ND_SensorResult_t* r)
{
    return (r != NULL) && (r->adc != 0xFFFFu);
}

static bool prv_sensor_pulse_valid(const ND_SensorResult_t* r)
{
    return (r != NULL) && (r->pulse_cnt != 0xFFFFFFFFu);
}

static bool prv_sensor_temp_valid(const ND_SensorResult_t* r)
{
    return (r != NULL) && (r->temp_c != UI_NODE_TEMP_INVALID_C);
}

static bool prv_sensor_batt_valid(const ND_SensorResult_t* r)
{
    return (r != NULL) && (r->batt_lvl != UI_NODE_BATT_LVL_INVALID);
}

static bool prv_sensor_result_has_any_valid(const ND_SensorResult_t* r, uint8_t cfg_mask)
{
    if (r == NULL) {
        return false;
    }

    if (prv_sensor_batt_valid(r) || prv_sensor_temp_valid(r)) {
        return true;
    }
    if (((cfg_mask & UI_SENSOR_EN_ICM20948) != 0u) && prv_sensor_xyz_valid(r)) {
        return true;
    }
    if (((cfg_mask & UI_SENSOR_EN_ADC) != 0u) && prv_sensor_adc_valid(r)) {
        return true;
    }
    if (((cfg_mask & UI_SENSOR_EN_PULSE) != 0u) && prv_sensor_pulse_valid(r)) {
        return true;
    }

    return false;
}

static uint8_t prv_build_effective_sensor_mask(const ND_SensorResult_t* r, uint8_t cfg_mask)
{
    uint8_t mask = (uint8_t)(cfg_mask & UI_SENSOR_EN_ALL);

    if (r == NULL) {
        return 0u;
    }

    if (!prv_sensor_xyz_valid(r)) {
        mask &= (uint8_t)~UI_SENSOR_EN_ICM20948;
    }
    if (!prv_sensor_adc_valid(r)) {
        mask &= (uint8_t)~UI_SENSOR_EN_ADC;
    }
    if (!prv_sensor_pulse_valid(r)) {
        mask &= (uint8_t)~UI_SENSOR_EN_PULSE;
    }

    return mask;
}

static void prv_merge_sensor_result(ND_SensorResult_t* dst, const ND_SensorResult_t* fresh, uint8_t cfg_mask)
{
    if ((dst == NULL) || (fresh == NULL)) {
        return;
    }

    if (prv_sensor_batt_valid(fresh)) {
        dst->batt_lvl = fresh->batt_lvl;
    }

    if (prv_sensor_temp_valid(fresh)) {
        dst->temp_c = fresh->temp_c;
    }

    if ((cfg_mask & UI_SENSOR_EN_ICM20948) != 0u) {
        if (prv_sensor_xyz_valid(fresh)) {
            dst->x = fresh->x;
            dst->y = fresh->y;
            dst->z = fresh->z;
        }
    } else {
        dst->x = (int16_t)0xFFFFu;
        dst->y = (int16_t)0xFFFFu;
        dst->z = (int16_t)0xFFFFu;
    }

    if ((cfg_mask & UI_SENSOR_EN_ADC) != 0u) {
        if (prv_sensor_adc_valid(fresh)) {
            dst->adc = fresh->adc;
        }
    } else {
        dst->adc = 0xFFFFu;
    }

    if ((cfg_mask & UI_SENSOR_EN_PULSE) != 0u) {
        if (prv_sensor_pulse_valid(fresh)) {
            dst->pulse_cnt = fresh->pulse_cnt;
        }
    } else {
        dst->pulse_cnt = 0xFFFFFFFFu;
    }
}

static void prv_fill_node_data_from_sensor(UI_NodeData_t* nd,
                                           const UI_Config_t* cfg,
                                           const ND_SensorResult_t* sensor,
                                           uint16_t beacon_cnt)
{
    uint8_t cfg_mask;

    if ((nd == NULL) || (cfg == NULL) || (sensor == NULL)) {
        return;
    }

    cfg_mask = (uint8_t)(cfg->sensor_en_mask & UI_SENSOR_EN_ALL);

    memset(nd, 0, sizeof(*nd));
    nd->node_num = cfg->node_num;
    memcpy(nd->net_id, cfg->net_id, UI_NET_ID_LEN);
    nd->batt_lvl = sensor->batt_lvl;
    nd->temp_c = sensor->temp_c;
    nd->beacon_cnt = beacon_cnt;

    if (((cfg_mask & UI_SENSOR_EN_ICM20948) != 0u) && prv_sensor_xyz_valid(sensor)) {
        nd->x = prv_encode_payload_x(sensor->x);
        nd->y = prv_encode_payload_y(sensor->y);
        nd->z = prv_encode_payload_z(sensor->z);
    } else {
        nd->x = UI_NODE_MEAS_UNUSED_U16;
        nd->y = UI_NODE_MEAS_UNUSED_U16;
        nd->z = UI_NODE_MEAS_UNUSED_U16;
    }

    if (((cfg_mask & UI_SENSOR_EN_ADC) != 0u) && prv_sensor_adc_valid(sensor)) {
        nd->adc = prv_encode_payload_adc(sensor->adc);
    } else {
        nd->adc = UI_NODE_MEAS_UNUSED_U16;
    }

    if ((cfg_mask & UI_SENSOR_EN_PULSE) != 0u) {
        nd->pulse_cnt = sensor->pulse_cnt;
    } else {
        nd->pulse_cnt = 0xFFFFFFFFu;
    }

    nd->sensor_en_mask = prv_build_effective_sensor_mask(sensor, cfg_mask);
}

static bool prv_node_data_equal(const UI_NodeData_t* a, const UI_NodeData_t* b)
{
    if ((a == NULL) || (b == NULL)) {
        return false;
    }

    return (a->node_num == b->node_num)
        && (memcmp(a->net_id, b->net_id, UI_NET_ID_LEN) == 0)
        && (a->batt_lvl == b->batt_lvl)
        && (a->temp_c == b->temp_c)
        && (a->beacon_cnt == b->beacon_cnt)
        && (a->x == b->x)
        && (a->y == b->y)
        && (a->z == b->z)
        && (a->adc == b->adc)
        && (a->pulse_cnt == b->pulse_cnt)
        && (a->sensor_en_mask == b->sensor_en_mask);
}

static bool prv_sensor_result_matches_node_data(const ND_SensorResult_t* sensor,
                                                const UI_NodeData_t* nd,
                                                uint8_t cfg_mask)
{
    uint16_t exp_x = UI_NODE_MEAS_UNUSED_U16;
    uint16_t exp_y = UI_NODE_MEAS_UNUSED_U16;
    uint16_t exp_z = UI_NODE_MEAS_UNUSED_U16;
    uint16_t exp_adc = UI_NODE_MEAS_UNUSED_U16;

    if ((sensor == NULL) || (nd == NULL)) {
        return false;
    }

    if ((sensor->batt_lvl != nd->batt_lvl) ||
        (sensor->temp_c != nd->temp_c)) {
        return false;
    }

    if (((cfg_mask & UI_SENSOR_EN_ICM20948) != 0u) && prv_sensor_xyz_valid(sensor)) {
        exp_x = prv_encode_payload_x(sensor->x);
        exp_y = prv_encode_payload_y(sensor->y);
        exp_z = prv_encode_payload_z(sensor->z);
    }

    if ((nd->x != exp_x) || (nd->y != exp_y) || (nd->z != exp_z)) {
        return false;
    }

    if (((cfg_mask & UI_SENSOR_EN_ADC) != 0u) && prv_sensor_adc_valid(sensor)) {
        exp_adc = prv_encode_payload_adc(sensor->adc);
    }
    if (nd->adc != exp_adc) {
        return false;
    }

    if ((cfg_mask & UI_SENSOR_EN_PULSE) != 0u) {
        if (sensor->pulse_cnt != nd->pulse_cnt) {
            return false;
        }
    } else if (nd->pulse_cnt != 0xFFFFFFFFu) {
        return false;
    }

    if (nd->sensor_en_mask != prv_build_effective_sensor_mask(sensor, cfg_mask)) {
        return false;
    }

    return true;
}

static bool prv_build_verified_node_payload(const UI_Config_t* cfg,
                                            const ND_SensorResult_t* sensor,
                                            uint16_t beacon_cnt,
                                            uint8_t out_payload[UI_NODE_PAYLOAD_LEN],
                                            uint8_t* out_payload_len,
                                            UI_NodeData_t* out_nd)
{
    UI_NodeData_t expected;
    UI_NodeData_t parsed;
    uint8_t cfg_mask;
    uint8_t payload_len;

    if ((cfg == NULL) || (sensor == NULL) || (out_payload == NULL)) {
        return false;
    }

    cfg_mask = (uint8_t)(cfg->sensor_en_mask & UI_SENSOR_EN_ALL);

    prv_fill_node_data_from_sensor(&expected, cfg, sensor, beacon_cnt);
    memset(out_payload, 0, UI_NODE_PAYLOAD_LEN);
    payload_len = UI_Pkt_BuildNodeData(out_payload, &expected);
    if ((payload_len == 0u) || (payload_len > UI_NODE_PAYLOAD_LEN)) {
        return false;
    }

    memset(&parsed, 0, sizeof(parsed));
    if (!UI_Pkt_ParseNodeData(out_payload, payload_len, &parsed)) {
        return false;
    }

    if (!prv_node_data_equal(&expected, &parsed)) {
        return false;
    }

    if (!prv_sensor_result_matches_node_data(sensor, &parsed, cfg_mask)) {
        return false;
    }

    if (out_payload_len != NULL) {
        *out_payload_len = payload_len;
    }
    if (out_nd != NULL) {
        *out_nd = parsed;
    }

    return true;
}

static bool prv_reload_sensor_snapshot_for_tx(uint8_t cfg_mask, ND_SensorResult_t* out_sensor)
{
    ND_SensorResult_t measured;
    ND_SensorResult_t merged;
    bool measure_ok;

    prv_set_invalid_sensor_result(&measured);
    measure_ok = ND_Sensors_MeasureAll(&measured, cfg_mask);
    if (!measure_ok) {
        return false;
    }

    if (s_last_sensor_valid) {
        merged = s_last_sensor;
    } else {
        prv_set_invalid_sensor_result(&merged);
    }

    prv_merge_sensor_result(&merged, &measured, cfg_mask);
    s_last_sensor = merged;
    s_last_sensor_valid = prv_sensor_result_has_any_valid(&s_last_sensor, cfg_mask);
    s_sensor_ready = s_last_sensor_valid;

    if (!s_last_sensor_valid) {
        return false;
    }

    if (out_sensor != NULL) {
        *out_sensor = s_last_sensor;
    }

    return true;
}

static void prv_send_test_result_ble(const ND_SensorResult_t* r)
{
    const char* batt_text;
    char msg[192];
    uint16_t sx = UI_NODE_MEAS_UNUSED_U16;
    uint16_t sy = UI_NODE_MEAS_UNUSED_U16;
    uint16_t sz = UI_NODE_MEAS_UNUSED_U16;
    uint16_t sa = UI_NODE_MEAS_UNUSED_U16;

    if (r == NULL) {
        return;
    }
    if (!UI_BLE_IsActive()) {
        return;
    }

    if (prv_sensor_xyz_valid(r)) {
        sx = prv_encode_payload_x(r->x);
        sy = prv_encode_payload_y(r->y);
        sz = prv_encode_payload_z(r->z);
    }
    if (prv_sensor_adc_valid(r)) {
        sa = prv_encode_payload_adc(r->adc);
    }

    batt_text = prv_batt_text_from_level(r->batt_lvl);
    (void)snprintf(msg, sizeof(msg),
                   "<TEST START:BATT:%s,T:%d,X:%u,Y:%u,Z:%u,A:%u,P:%lu>\r\n",
                   batt_text,
                   (int)r->temp_c,
                   (unsigned)sx,
                   (unsigned)sy,
                   (unsigned)sz,
                   (unsigned)sa,
                   (unsigned long)r->pulse_cnt);
    UI_UART_SendString(msg);
}

static bool prv_build_sync_request_payload(uint8_t out_payload[UI_NODE_PAYLOAD_LEN],
                                           uint8_t* out_payload_len)
{
    const UI_Config_t *cfg = UI_GetConfig();
    UI_NodeData_t nd;
    uint8_t payload_len;

    if ((cfg == NULL) || (out_payload == NULL)) {
        return false;
    }

    memset(&nd, 0, sizeof(nd));
    nd.node_num = 0xAAu;
    memcpy(nd.net_id, cfg->net_id, UI_NET_ID_LEN);
    nd.batt_lvl = UI_NODE_BATT_LVL_INVALID;
    nd.temp_c = UI_NODE_TEMP_INVALID_C;
    nd.beacon_cnt = s_beacon_cnt;
    nd.x = UI_NODE_MEAS_UNUSED_U16;
    nd.y = UI_NODE_MEAS_UNUSED_U16;
    nd.z = UI_NODE_MEAS_UNUSED_U16;
    nd.adc = UI_NODE_MEAS_UNUSED_U16;
    nd.pulse_cnt = 0xFFFFFFFFu;
    nd.sensor_en_mask = 0u;
    payload_len = UI_Pkt_BuildNodeData(out_payload, &nd);
    if ((payload_len == 0u) || (payload_len > UI_NODE_PAYLOAD_LEN)) {
        return false;
    }
    if (out_payload_len != NULL) {
        *out_payload_len = payload_len;
    }
    return true;
}

static bool prv_start_sync_request_tx(void)
{
    if ((s_state == ND_STATE_RX_BEACON) || (s_state == ND_STATE_TX_DATA)) {
        prv_abort_active_radio_for_ble();
    }
    if (s_state != ND_STATE_IDLE) {
        return false;
    }
    if (!prv_build_sync_request_payload(s_sync_req_tx_payload, &s_sync_req_tx_payload_len)) {
        return false;
    }
    prv_stop_sensor_and_tx_timers();
    prv_clear_pending_runtime_rx_events();
    (void)UTIL_TIMER_Stop(&s_tmr_boot_listen);
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    prv_boot_led_blink_stop();
    s_sensor_ready = false;
    s_sync_cmd_active = true;
    s_sync_tx_wait_beacon_pending = false;

    prv_hold_ble_for_sync();

    if (!prv_radio_ready_for_tx()) {
        UI_Radio_MarkRecoverNeeded();
        if (Radio.Sleep != NULL) {
            Radio.Sleep();
        }
        s_sync_cmd_active = false;
        return false;
    }
    if ((s_sync_req_tx_payload_len == 0u) || !UI_Radio_PrepareTx(s_sync_req_tx_payload_len)) {
        UI_Radio_MarkRecoverNeeded();
        if (Radio.Sleep != NULL) {
            Radio.Sleep();
        }
        s_sync_cmd_active = false;
        return false;
    }

    UI_LPM_LockStop();
    s_state = ND_STATE_TX_DATA;
    s_tx_inflight_slot_id = 0xFFFFFFFFu;
    s_sync_tx_wait_beacon_pending = true;
    prv_start_tx_watchdog();
    Radio.SetChannel(UI_RF_GetBeaconFreqHz());
    Radio.Send(s_sync_req_tx_payload, s_sync_req_tx_payload_len);
    return true;
}

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

static void prv_tmr_tx_watchdog_cb(void *context)
{
    (void)context;
    s_evt_flags |= ND_EVT_TX_RECOVER;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_tmr_test_session_cb(void *context)
{
    (void)context;
    s_evt_flags |= ND_EVT_TEST_SESSION_EXPIRE;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_stop_tx_watchdog(void)
{
    (void)UTIL_TIMER_Stop(&s_tmr_tx_watchdog);
}

static void prv_start_tx_watchdog(void)
{
    (void)UTIL_TIMER_Stop(&s_tmr_tx_watchdog);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_tx_watchdog, ND_TX_WATCHDOG_MS);
    (void)UTIL_TIMER_Start(&s_tmr_tx_watchdog);
}

static void prv_request_stop_mode(void)
{
    s_evt_flags |= ND_EVT_ENTER_STOP;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_enter_stop_now_if_possible(void)
{
    uint32_t pending_mask = (s_evt_flags & ~ND_EVT_ENTER_STOP);

    if (pending_mask != 0u) {
        return;
    }
    if (UI_BLE_IsActive()) {
        return;
    }
    if (s_state != ND_STATE_IDLE) {
        return;
    }

    s_evt_flags &= ~ND_EVT_ENTER_STOP;
    (void)UTIL_TIMER_Stop(&s_tmr_led1_pulse);
    prv_led0(false);
    prv_led1(false);
    if (Radio.Sleep != NULL) {
        Radio.Sleep();
    }
    UI_LPM_EnterStopNow();
}

static void prv_request_stop_mode_if_possible(void)
{
    /*
     * STOP2는 ND main task 문맥에서만 진입시킨다.
     *
     * 이 함수는 radio callback / RTC timer callback 뒤 경로에서도 호출될 수 있으므로
     * 여기서 바로 prv_enter_stop_now_if_possible()를 호출하면 ISR 문맥에서 STOP2로
     * 들어가게 될 수 있다.
     *
     * 부팅 직후 beacon 수신 다음에 주기 beacon/sensor/TX가 멈추는 현상을 피하기 위해
     * 여기서는 직접 진입하지 않고 ND_EVT_ENTER_STOP만 올려서
     * ND_App_Process()가 task 문맥에서 안전하게 처리하도록 고정한다.
     */
    if ((s_evt_flags & ND_EVT_ENTER_STOP) != 0u) {
        return;
    }
    if (s_evt_flags != 0u) {
        return;
    }
    if (UI_BLE_IsActive()) {
        return;
    }
    if (s_state != ND_STATE_IDLE) {
        return;
    }

    prv_request_stop_mode();
}

static void prv_abort_active_radio_for_ble(void)
{
    if (s_state == ND_STATE_TX_DATA) {
        prv_stop_tx_watchdog();
        UI_Radio_MarkRecoverNeeded();
        if (Radio.Sleep != NULL) {
            Radio.Sleep();
        }
        s_state = ND_STATE_IDLE;
        s_tx_inflight_slot_id = 0xFFFFFFFFu;
        UI_LPM_UnlockStop();
        return;
    }

    if (s_state == ND_STATE_RX_BEACON) {
        if (Radio.Sleep != NULL) {
            Radio.Sleep();
        }
        s_state = ND_STATE_IDLE;
        s_rx_reason = ND_RX_REASON_NONE;
        s_rx_window_deadline_ms = 0u;
        UI_LPM_UnlockStop();
    }
}

static void prv_enter_unsynced_idle(void)
{
    s_sync_state = ND_SYNC_BOOT_SEARCH;
    s_beacon_ok = false;
    s_runtime_enabled = false;
    s_sensor_ready = false;
    prv_boot_led_blink_stop();
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    s_boot_listen_stop_after_ble = false;
    s_phase_walk_idx = 0u;
    s_force_gw_phase_scan = false;
    s_gw_phase_scan_cycle_count = 0u;
    (void)UTIL_TIMER_Stop(&s_tmr_boot_listen);
    prv_clear_pending_runtime_rx_events();
    prv_stop_sensor_and_tx_timers();
    prv_request_stop_mode_if_possible();
}

static void prv_resume_schedule_after_boot_search(void)
{
    if (prv_try_enter_holdover_from_backup()) {
        prv_continue_boot_listen_or_schedule();
        return;
    }

    if (!UI_Time_IsValid()) {
        prv_enter_unsynced_idle();
        return;
    }

    s_sync_state = ND_SYNC_HOLDOVER;
    s_beacon_ok = false;
    s_runtime_enabled = true;
    s_sensor_ready = false;
    s_phase_walk_idx = 0u;
    s_boot_listen_stop_after_ble = false;
    s_force_gw_phase_scan = false;
    s_gw_phase_scan_cycle_count = 0u;
    if (s_beacon_miss_count < 1u) {
        s_beacon_miss_count = 1u;
    }
    prv_reset_unsync_backoff();
    prv_continue_boot_listen_or_schedule();
}

static void prv_clear_pending_runtime_rx_events(void)
{
    s_evt_flags &= ~(ND_EVT_BEACON_LISTEN_START | ND_EVT_REMINDER_LISTEN_START);
    (void)UTIL_TIMER_Stop(&s_tmr_beacon_sched);
    (void)UTIL_TIMER_Stop(&s_tmr_reminder_sched);
}

static bool prv_abort_beacon_rx_for_tx(void)
{
    if (s_state != ND_STATE_RX_BEACON) {
        return false;
    }

    if (Radio.Sleep != NULL) {
        Radio.Sleep();
    }

    s_state = ND_STATE_IDLE;
    s_rx_reason = ND_RX_REASON_NONE;
    s_rx_window_deadline_ms = 0u;
    UI_LPM_UnlockStop();
    return true;
}

static void prv_suspend_runtime_rx_around_tx(void)
{
    if (!s_boot_listen_active) {
        prv_clear_pending_runtime_rx_events();
    }
}

static void prv_refresh_runtime_timers_after_tx(void)
{
    prv_clear_pending_runtime_rx_events();
    prv_continue_boot_listen_or_schedule();
    if (s_runtime_enabled) {
        prv_schedule_reminder_window();
        prv_schedule_sensor_and_tx();
    } else {
        prv_stop_sensor_and_tx_timers();
    }
}

static void prv_force_tx_recovery(bool keep_last_slot)
{
    prv_stop_tx_watchdog();
    UI_Radio_MarkRecoverNeeded();
    if (Radio.Sleep != NULL) {
        Radio.Sleep();
    }
    s_state = ND_STATE_IDLE;
    if (keep_last_slot && (s_tx_inflight_slot_id != 0xFFFFFFFFu)) {
        s_last_tx_slot_id = s_tx_inflight_slot_id;
    }
    s_tx_inflight_slot_id = 0xFFFFFFFFu;
    UI_LPM_UnlockStop();
    prv_refresh_runtime_timers_after_tx();
}

static bool prv_format_setting_ascii(uint8_t value, char unit, uint8_t out_setting_ascii[3])
{
    if (out_setting_ascii == NULL) {
        return false;
    }
    if ((value > 99u) || ((unit != 'M') && (unit != 'H'))) {
        return false;
    }

    out_setting_ascii[0] = (uint8_t)('0' + ((value / 10u) % 10u));
    out_setting_ascii[1] = (uint8_t)('0' + (value % 10u));
    out_setting_ascii[2] = (uint8_t)unit;
    return true;
}

static void prv_request_immediate_beacon_scan(void)
{
    s_force_gw_phase_scan = true;
    s_evt_flags |= ND_EVT_BEACON_LISTEN_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

static void prv_start_test_session_from_cmd(void)
{
    const UI_Config_t *cfg = UI_GetConfig();

    if (!s_test_session_active && (cfg != NULL)) {
        s_test_session_restore_value = cfg->setting_value;
        s_test_session_restore_unit = cfg->setting_unit;
    }

    s_test_session_active = true;
    (void)UTIL_TIMER_Stop(&s_tmr_test_session);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_test_session, ND_TEST_SESSION_MS);
    (void)UTIL_TIMER_Start(&s_tmr_test_session);

    /* TEST START 직후에는 저장 없이 임시 1H로 두고, 실제 주기는 beacon setting을 따른다. */
    UI_SetSetting(1u, 'H');
    prv_refresh_mode_from_config();
    prv_request_immediate_beacon_scan();
}

static void prv_stop_test_session(void)
{
    if (!s_test_session_active) {
        return;
    }

    s_test_session_active = false;
    (void)UTIL_TIMER_Stop(&s_tmr_test_session);
    UI_SetSetting(s_test_session_restore_value, s_test_session_restore_unit);
    prv_refresh_mode_from_config();

    if (s_state == ND_STATE_IDLE) {
        prv_continue_boot_listen_or_schedule();
    }
}


static void prv_apply_setting_ascii(const uint8_t setting_ascii[3])
{
    uint8_t d0 = setting_ascii[0];
    uint8_t d1 = setting_ascii[1];
    char unit = (char)setting_ascii[2];

    if ((d0 < '0') || (d0 > '9') || (d1 < '0') || (d1 > '9') || ((unit != 'M') && (unit != 'H'))) {
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
    const UI_Config_t *cfg = UI_GetConfig();
    s_test_mode = ((cfg->setting_value == 1u) && (cfg->setting_unit == 'M'));
}

static bool prv_is_two_minute_mode_active(void)
{
    const UI_Config_t *cfg = UI_GetConfig();
    return ((cfg->setting_value == 2u) && (cfg->setting_unit == 'M'));
}

static uint32_t prv_get_setting_cycle_sec(void)
{
    const UI_Config_t *cfg = UI_GetConfig();

    if ((cfg->setting_value == 0u) || ((cfg->setting_unit != 'M') && (cfg->setting_unit != 'H'))) {
        return 0u;
    }
    if (cfg->setting_unit == 'M') {
        return (uint32_t)cfg->setting_value * 60u;
    }
    return (uint32_t)cfg->setting_value * 3600u;
}

static uint32_t prv_get_normal_cycle_sec(void)
{
    uint32_t cycle_sec = prv_get_setting_cycle_sec();
    if (cycle_sec == 0u) {
        return 3600u;
    }
    if (prv_is_two_minute_mode_active()) {
        return 120u;
    }
    if (cycle_sec < UI_BEACON_PERIOD_S) {
        return UI_BEACON_PERIOD_S;
    }
    return cycle_sec;
}

static uint32_t prv_get_beacon_interval_sec(void)
{
    if (s_test_mode) {
        return 60u;
    }
    if (prv_is_two_minute_mode_active()) {
        return 120u;
    }
    return UI_BEACON_PERIOD_S;
}

static uint64_t prv_next_event_centi(uint64_t now_centi, uint32_t interval_sec, uint32_t offset_sec)
{
    uint32_t now_sec = (uint32_t)(now_centi / 100u);
    uint32_t centi = (uint32_t)(now_centi % 100u);
    uint32_t cand_sec = now_sec + ((centi == 0u) ? 0u : 1u);

    if (interval_sec == 0u) {
        return (uint64_t)(cand_sec + offset_sec) * 100u;
    }

    uint32_t rem = cand_sec % interval_sec;
    uint32_t next_sec = (rem <= offset_sec)
                        ? (cand_sec - rem + offset_sec)
                        : (cand_sec - rem + interval_sec + offset_sec);
    return (uint64_t)next_sec * 100u;
}

static bool prv_is_event_due_now(uint64_t now_centi,
                                 uint32_t interval_sec,
                                 uint32_t offset_sec,
                                 uint32_t late_grace_centi,
                                 uint64_t *due_event_centi)
{
    uint64_t next_evt;
    uint64_t step;
    uint64_t prev_evt;

    if ((interval_sec == 0u) || (due_event_centi == NULL)) {
        return false;
    }

    next_evt = prv_next_event_centi(now_centi, interval_sec, offset_sec);
    step = (uint64_t)interval_sec * 100u;

    if (next_evt == now_centi) {
        *due_event_centi = next_evt;
        return true;
    }
    if (next_evt < step) {
        return false;
    }

    prev_evt = next_evt - step;
    if ((now_centi >= prev_evt) &&
        (now_centi < (prev_evt + (uint64_t)late_grace_centi))) {
        *due_event_centi = prev_evt;
        return true;
    }

    return false;
}

static uint32_t prv_get_tx_base_offset_sec(void)
{
    if (s_test_mode) {
        return 20u;
    }
    if (prv_is_two_minute_mode_active()) {
        return 20u;
    }
    return 60u;
}

static uint32_t prv_get_tx_in_slot_delay_ms(uint8_t node_num)
{
    uint32_t delay_ms = ND_TX_IN_SLOT_DELAY_MS;

    if (node_num == 0u) {
        delay_ms += ND_TX_SLOT0_EXTRA_DELAY_MS;
    }

    /* TX는 각 2초 슬롯 안에서 충분히 이른 시점에 끝나야 한다.
     * 지나치게 큰 지연값이 설정돼도 다음 슬롯을 침범하지 않도록 상한을 둔다. */
    if (delay_ms >= (UI_SLOT_DURATION_MS - ND_TX_RETRY_GUARD_MS - 50u)) {
        delay_ms = UI_SLOT_DURATION_MS - ND_TX_RETRY_GUARD_MS - 50u;
    }

    return delay_ms;
}

static void prv_schedule_tx_event_at(uint64_t target_centi, uint8_t node_num)
{
    uint64_t now_centi = UI_Time_NowCenti2016();
    uint32_t delay_ms = (uint32_t)((target_centi > now_centi) ? ((target_centi - now_centi) * 10u) : 1u);

    delay_ms += prv_get_tx_in_slot_delay_ms(node_num);
    if (delay_ms == 0u) {
        delay_ms = 1u;
    }

    (void)UTIL_TIMER_Stop(&s_tmr_tx_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_tx_sched, delay_ms);
    (void)UTIL_TIMER_Start(&s_tmr_tx_sched);
}

static uint32_t prv_periodic_slot_id_from_epoch_sec(uint32_t epoch_sec, uint32_t period_sec, uint32_t offset_sec)
{
    if (period_sec == 0u) {
        return 0xFFFFFFFFu;
    }
    if (epoch_sec < offset_sec) {
        return 0u;
    }
    return ((epoch_sec - offset_sec) / period_sec);
}

static uint32_t prv_floor_cycle_anchor_sec(uint32_t epoch_sec, uint32_t period_sec)
{
    if (period_sec == 0u) {
        return epoch_sec;
    }

    /* Data TX/RX cycle은 beacon phase(00/02/04초)와 독립적인 절대 시각 기준이다.
     * beacon이 5분 reminder로 더 자주 와도, TX hop anchor는 실제 cycle 시작(분/시간 경계)
     * 기준으로 고정해야 GW RX data frequency와 일치한다. */
    return ((epoch_sec / period_sec) * period_sec);
}

static uint32_t prv_get_data_freq_anchor_sec(uint32_t now_sec)
{
    uint32_t period_sec = prv_get_runtime_cycle_sec();
    uint32_t cycle_anchor_sec = prv_floor_cycle_anchor_sec(now_sec, period_sec);
    return cycle_anchor_sec + prv_get_tx_base_offset_sec();
}

static bool prv_get_next_predicted_beacon_centi(uint64_t now_centi, uint64_t *out_next_centi)
{
    uint32_t interval_sec;
    uint64_t anchor_centi;
    uint64_t step_centi;
    uint64_t elapsed;
    uint64_t steps;

    if (out_next_centi == NULL) {
        return false;
    }

    interval_sec = prv_get_beacon_interval_sec();
    if ((interval_sec == 0u) || (s_last_beacon_anchor_sec == 0u)) {
        return false;
    }

    anchor_centi = (uint64_t)s_last_beacon_anchor_sec * 100u;
    step_centi = (uint64_t)interval_sec * 100u;

    if (now_centi < anchor_centi) {
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
    uint32_t max = prv_is_two_minute_mode_active() ? ND_BEACON_EARLY_WAKE_MAX_MS_2M : ND_BEACON_EARLY_WAKE_MAX_MS;
    uint32_t extra = (uint32_t)s_beacon_miss_count * ND_BEACON_EARLY_WAKE_STEP_MS;
    uint32_t early = base + extra;
    if (early > max) {
        early = max;
    }
    return early;
}

static uint32_t prv_get_beacon_window_ms(void)
{
    uint32_t early = prv_get_beacon_early_wake_ms();
    uint32_t tail = prv_is_two_minute_mode_active() ? ND_BEACON_TAIL_MS_2M : ND_BEACON_TAIL_MS_NORMAL;
    return early + tail;
}

static bool prv_select_next_gw_phase_scan(void)
{
    if (s_boot_listen_active || (s_sync_state == ND_SYNC_UNSYNC_SEARCH)) {
        return true;
    }

    s_gw_phase_scan_cycle_count++;
    if (s_gw_phase_scan_cycle_count >= ND_GW_PHASE_SCAN_EVERY_N_CYCLES) {
        s_gw_phase_scan_cycle_count = 0u;
        return true;
    }
    return false;
}

static uint32_t prv_get_beacon_rx_window_ms_with_phase_scan(void)
{
    uint32_t window_ms = prv_get_beacon_window_ms();
    if (s_force_gw_phase_scan) {
        window_ms += ND_GW_PHASE_SCAN_EXTRA_MS;
    }
    return window_ms;
}

static uint32_t prv_get_unsync_backoff_beacon_count(void)
{
    uint32_t count = (uint32_t)(sizeof(k_unsync_backoff_beacon_count) / sizeof(k_unsync_backoff_beacon_count[0]));
    uint32_t idx = s_unsync_backoff_idx;

    if (count == 0u) {
        return 144u;
    }
    if (idx >= count) {
        idx = count - 1u;
    }
    return (uint32_t)k_unsync_backoff_beacon_count[idx];
}

static uint32_t prv_get_unsync_backoff_sec(void)
{
    uint32_t beacon_interval_sec = prv_get_beacon_interval_sec();

    if (beacon_interval_sec == 0u) {
        beacon_interval_sec = UI_BEACON_PERIOD_S;
    }
    return prv_get_unsync_backoff_beacon_count() * beacon_interval_sec;
}

static void prv_reset_unsync_backoff(void)
{
    s_unsync_backoff_idx = 0u;
}

static void prv_advance_unsync_backoff(void)
{
    uint32_t count = (uint32_t)(sizeof(k_unsync_backoff_beacon_count) / sizeof(k_unsync_backoff_beacon_count[0]));

    if ((count > 0u) && ((uint32_t)s_unsync_backoff_idx < (count - 1u))) {
        s_unsync_backoff_idx++;
    }
}

static uint32_t prv_get_unsync_search_delay_ms(void)
{
    return prv_get_unsync_backoff_sec() * 1000u;
}

static uint32_t prv_get_runtime_cycle_sec(void)
{
    if (s_test_mode) {
        return 60u;
    }
    if (s_sync_state == ND_SYNC_UNSYNC_SEARCH) {
        return prv_get_unsync_backoff_sec();
    }
    return prv_get_normal_cycle_sec();
}

static int32_t prv_get_phase_walk_offset_sec(void)
{
    static const int16_t k_offsets_sec[] = { -20, 20, -40, 40, -60, 60, -90, 90 };
    uint32_t count = (uint32_t)(sizeof(k_offsets_sec) / sizeof(k_offsets_sec[0]));

    return (count > 0u) ? (int32_t)k_offsets_sec[s_phase_walk_idx % count] : 0;
}

static void prv_advance_phase_walk_index(void)
{
    static const int16_t k_offsets_sec[] = { -20, 20, -40, 40, -60, 60, -90, 90 };
    uint32_t count = (uint32_t)(sizeof(k_offsets_sec) / sizeof(k_offsets_sec[0]));

    if (count > 0u) {
        s_phase_walk_idx = (uint8_t)((s_phase_walk_idx + 1u) % count);
    }
}

static bool prv_get_next_phase_walk_scan_centi(uint64_t now_centi, uint64_t *out_scan_centi)
{
    uint64_t next_predicted_centi = 0u;
    int64_t scan_centi;

    if ((out_scan_centi == NULL) || !prv_get_next_predicted_beacon_centi(now_centi, &next_predicted_centi)) {
        return false;
    }

    scan_centi = (int64_t)next_predicted_centi + ((int64_t)prv_get_phase_walk_offset_sec() * 100ll);
    if (scan_centi <= (int64_t)now_centi) {
        *out_scan_centi = now_centi + 1u;
    } else {
        *out_scan_centi = (uint64_t)scan_centi;
    }
    return true;
}

static uint32_t prv_pack_sync_meta(const uint8_t setting_ascii[3])
{
    return ((uint32_t)setting_ascii[0]) |
           ((uint32_t)setting_ascii[1] << 8u) |
           ((uint32_t)setting_ascii[2] << 16u);
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

static void prv_save_sync_anchor_from_beacon(const UI_Beacon_t *beacon)
{
    uint32_t epoch_sec;
    uint32_t meta;
    uint16_t crc;
    const UI_Config_t *cfg;
    uint8_t persist_setting_ascii[3];

    if (beacon == NULL) {
        return;
    }

    cfg = UI_GetConfig();
    epoch_sec = UI_Time_Epoch2016_FromCalendar(&beacon->dt);
    if (s_test_session_active) {
        if (!prv_format_setting_ascii(s_test_session_restore_value, s_test_session_restore_unit, persist_setting_ascii)) {
            return;
        }
        meta = prv_pack_sync_meta(persist_setting_ascii);
    } else {
        meta = prv_pack_sync_meta(beacon->setting_ascii);
    }
    crc = prv_calc_sync_anchor_crc(epoch_sec, meta, cfg->net_id);

    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, ND_SYNC_BKP_DR_MAGIC, ND_SYNC_BKP_MAGIC);
    HAL_RTCEx_BKUPWrite(&hrtc, ND_SYNC_BKP_DR_EPOCH, epoch_sec);
    HAL_RTCEx_BKUPWrite(&hrtc, ND_SYNC_BKP_DR_META, meta);
    HAL_RTCEx_BKUPWrite(&hrtc, ND_SYNC_BKP_DR_CRC, (uint32_t)crc);
    s_last_beacon_anchor_sec = epoch_sec;
}

static bool prv_load_sync_anchor(uint32_t *out_epoch_sec, uint8_t out_setting_ascii[3])
{
    uint32_t magic;
    uint32_t epoch_sec;
    uint32_t meta;
    uint16_t crc_stored;
    uint16_t crc_expected;
    const UI_Config_t *cfg = UI_GetConfig();

    if ((!UI_Time_IsValid()) || (out_setting_ascii == NULL)) {
        return false;
    }

    magic = HAL_RTCEx_BKUPRead(&hrtc, ND_SYNC_BKP_DR_MAGIC);
    if (magic != ND_SYNC_BKP_MAGIC) {
        return false;
    }

    epoch_sec = HAL_RTCEx_BKUPRead(&hrtc, ND_SYNC_BKP_DR_EPOCH);
    meta = HAL_RTCEx_BKUPRead(&hrtc, ND_SYNC_BKP_DR_META);
    crc_stored = (uint16_t)(HAL_RTCEx_BKUPRead(&hrtc, ND_SYNC_BKP_DR_CRC) & 0xFFFFu);

    prv_unpack_sync_meta(meta, out_setting_ascii);
    if (!prv_setting_ascii_is_valid(out_setting_ascii)) {
        return false;
    }

    crc_expected = prv_calc_sync_anchor_crc(epoch_sec, meta, cfg->net_id);
    if (crc_expected != crc_stored) {
        return false;
    }

    if (out_epoch_sec != NULL) {
        *out_epoch_sec = epoch_sec;
    }
    return true;
}

static void prv_enter_locked_from_beacon(const UI_Beacon_t *beacon)
{
    prv_save_sync_anchor_from_beacon(beacon);
    s_sync_state = ND_SYNC_LOCKED;
    s_beacon_ok = true;
    s_beacon_miss_count = 0u;
    prv_reset_unsync_backoff();
    s_phase_walk_idx = 0u;
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    prv_boot_led_blink_stop();
    s_runtime_enabled = true;
    s_sensor_ready = false;
}

static bool prv_try_enter_holdover_from_backup(void)
{
    uint8_t setting_ascii[3];
    uint32_t epoch_sec = 0u;

    if (!prv_load_sync_anchor(&epoch_sec, setting_ascii)) {
        return false;
    }

    prv_apply_setting_ascii(setting_ascii);
    s_last_beacon_anchor_sec = epoch_sec;
    s_sync_state = ND_SYNC_HOLDOVER;
    s_beacon_ok = false;
    s_phase_walk_idx = 0u;
    s_runtime_enabled = true;
    s_sensor_ready = false;
    if (s_beacon_miss_count < 1u) {
        s_beacon_miss_count = 1u;
    }
    prv_reset_unsync_backoff();
    return true;
}

static void prv_stop_sensor_and_tx_timers(void)
{
    (void)UTIL_TIMER_Stop(&s_tmr_sensor_sched);
    (void)UTIL_TIMER_Stop(&s_tmr_tx_sched);
    (void)UTIL_TIMER_Stop(&s_tmr_tx_watchdog);
    (void)UTIL_TIMER_Stop(&s_tmr_reminder_sched);
}

static void prv_schedule_unsync_search_scan(void)
{
    uint32_t delay_ms = prv_get_unsync_search_delay_ms();
    (void)UTIL_TIMER_Stop(&s_tmr_beacon_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_beacon_sched, delay_ms);
    (void)UTIL_TIMER_Start(&s_tmr_beacon_sched);
    prv_request_stop_mode_if_possible();
}

static void prv_enter_unsync_search(void)
{
    if (s_sync_state != ND_SYNC_UNSYNC_SEARCH) {
        prv_reset_unsync_backoff();
    }
    s_sync_state = ND_SYNC_UNSYNC_SEARCH;
    s_beacon_ok = false;
    s_runtime_enabled = UI_Time_IsValid();
    s_sensor_ready = false;
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    prv_boot_led_blink_stop();
    s_phase_walk_idx = 0u;
    s_force_gw_phase_scan = false;
    s_gw_phase_scan_cycle_count = 0u;
    if (s_beacon_miss_count < 1u) {
        s_beacon_miss_count = 1u;
    }
    (void)UTIL_TIMER_Stop(&s_tmr_reminder_sched);
    if (prv_should_pause_runtime_for_ble()) {
        prv_clear_pending_runtime_rx_events();
        prv_stop_sensor_and_tx_timers();
        return;
    }
    if (s_runtime_enabled) {
        prv_schedule_sensor_and_tx();
    } else {
        prv_stop_sensor_and_tx_timers();
    }
    prv_schedule_unsync_search_scan();
}

static void prv_enter_phase_walk(void)
{
    s_sync_state = ND_SYNC_PHASE_WALK;
    s_beacon_ok = false;
    s_runtime_enabled = true;
    s_sensor_ready = false;
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    prv_boot_led_blink_stop();
    s_force_gw_phase_scan = false;
}

static void prv_note_main_beacon_miss(void)
{
    bool can_phase_walk;

    s_beacon_ok = false;
    if (s_boot_listen_active) {
        return;
    }
    if ((s_sync_state == ND_SYNC_LOCKED) ||
        (s_sync_state == ND_SYNC_HOLDOVER) ||
        (s_sync_state == ND_SYNC_PHASE_WALK)) {
        if (s_beacon_miss_count < 250u) {
            s_beacon_miss_count++;
        }

        if (s_beacon_miss_count <= ND_SYNC_HOLDOVER_MAX_MISS) {
            s_sync_state = ND_SYNC_HOLDOVER;
            return;
        }

        can_phase_walk = UI_Time_IsValid() &&
                         (s_last_beacon_anchor_sec != 0u) &&
                         (s_beacon_miss_count <= ND_SYNC_PHASE_WALK_MAX_MISS);
        if (can_phase_walk) {
            if (s_sync_state != ND_SYNC_PHASE_WALK) {
                s_phase_walk_idx = 0u;
            }
            prv_enter_phase_walk();
            return;
        }

        prv_enter_unsync_search();
    }
}

static uint32_t prv_ms_until(uint32_t deadline_ms)
{
    uint32_t now = UTIL_TIMER_GetCurrentTime();
    return ((int32_t)(deadline_ms - now) > 0) ? (deadline_ms - now) : 0u;
}

static void prv_schedule_short_boot_retry(void)
{
    uint32_t remain_ms = prv_ms_until(s_boot_listen_deadline_ms);
    uint32_t retry_ms = ND_RX_RETRY_DELAY_MS;

    if (remain_ms == 0u) {
        prv_continue_boot_listen_or_schedule();
        return;
    }
    if (retry_ms > remain_ms) {
        retry_ms = remain_ms;
    }
    if (retry_ms == 0u) {
        retry_ms = 1u;
    }
    (void)UTIL_TIMER_Stop(&s_tmr_boot_listen);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_boot_listen, retry_ms);
    (void)UTIL_TIMER_Start(&s_tmr_boot_listen);
    prv_request_stop_mode_if_possible();
}

static void prv_schedule_short_beacon_retry(void)
{
    (void)UTIL_TIMER_Stop(&s_tmr_beacon_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_beacon_sched, ND_RX_RETRY_DELAY_MS);
    (void)UTIL_TIMER_Start(&s_tmr_beacon_sched);
    prv_request_stop_mode_if_possible();
}

static uint32_t prv_get_tx_slot_remaining_ms(uint64_t now_centi, uint32_t period_sec, uint32_t tx_off_sec)
{
    uint32_t now_sec;
    uint32_t tx_slot_id;
    uint64_t slot_start_centi;
    uint64_t slot_end_centi;

    if (period_sec == 0u) {
        return 0u;
    }

    now_sec = (uint32_t)(now_centi / 100u);
    tx_slot_id = prv_periodic_slot_id_from_epoch_sec(now_sec, period_sec, tx_off_sec);
    slot_start_centi = (((uint64_t)tx_slot_id * (uint64_t)period_sec) + (uint64_t)tx_off_sec) * 100u;
    slot_end_centi = slot_start_centi + ((uint64_t)UI_SLOT_DURATION_MS / 10u);

    if (slot_end_centi <= now_centi) {
        return 0u;
    }

    return (uint32_t)((slot_end_centi - now_centi) * 10u);
}

static bool prv_schedule_short_tx_retry(uint32_t period_sec, uint32_t tx_off_sec)
{
    uint64_t now_centi = UI_Time_NowCenti2016();
    uint32_t remain_ms = prv_get_tx_slot_remaining_ms(now_centi, period_sec, tx_off_sec);
    uint32_t retry_ms = ND_TX_RETRY_DELAY_MS;

    if (remain_ms <= ND_TX_RETRY_GUARD_MS) {
        return false;
    }

    remain_ms -= ND_TX_RETRY_GUARD_MS;
    if (retry_ms > remain_ms) {
        retry_ms = remain_ms;
    }
    if (retry_ms == 0u) {
        retry_ms = 1u;
    }

    (void)UTIL_TIMER_Stop(&s_tmr_tx_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_tx_sched, retry_ms);
    (void)UTIL_TIMER_Start(&s_tmr_tx_sched);
    prv_request_stop_mode_if_possible();
    return true;
}

static bool prv_get_tx_retry_params(uint32_t *out_period_sec, uint32_t *out_tx_off_sec)
{
    const UI_Config_t *cfg;

    if ((out_period_sec == NULL) || (out_tx_off_sec == NULL)) {
        return false;
    }

    cfg = UI_GetConfig();
    if (cfg == NULL) {
        return false;
    }

    *out_period_sec = prv_get_runtime_cycle_sec();
    *out_tx_off_sec = prv_get_tx_base_offset_sec() + (uint32_t)cfg->node_num * 2u;
    return (*out_period_sec != 0u);
}

static void prv_reschedule_main_if_pending(void)
{
    if (s_evt_flags != 0u) {
        UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
        return;
    }

    prv_request_stop_mode_if_possible();
}

static bool prv_start_beacon_rx(uint32_t window_ms, ND_RxReason_t reason)
{
    if (s_state != ND_STATE_IDLE) {
        return false;
    }
    if (window_ms == 0u) {
        window_ms = 1u;
    }
    if (!prv_radio_ready_for_rx()) {
        return false;
    }
    if (!UI_Radio_PrepareRx(UI_BEACON_PAYLOAD_LEN)) {
        return false;
    }

    if (reason != ND_RX_REASON_REMINDER) {
        s_beacon_ok = false;
    }

    s_rx_window_deadline_ms = UTIL_TIMER_GetCurrentTime() + window_ms;
    s_rx_reason = reason;
    UI_LPM_LockStop();
    s_state = ND_STATE_RX_BEACON;
    Radio.SetChannel(UI_RF_GetBeaconFreqHz());
    Radio.Rx(window_ms);
    return true;
}

static void prv_begin_boot_listen(void)
{
    uint32_t first_ms;

    s_sync_state = ND_SYNC_BOOT_SEARCH;
    s_runtime_enabled = false;
    s_sensor_ready = false;
    s_beacon_ok = false;
    s_phase_walk_idx = 0u;
    s_boot_listen_stop_after_ble = false;
    s_boot_listen_active = true;
    s_boot_listen_deadline_ms = UTIL_TIMER_GetCurrentTime() + UI_ND_BOOT_LISTEN_MS;
    prv_stop_sensor_and_tx_timers();

    first_ms = UI_ND_BOOT_LISTEN_MS;
    if (first_ms > ND_BOOT_RX_WINDOW_MS) {
        first_ms = ND_BOOT_RX_WINDOW_MS;
    }
    if (first_ms == 0u) {
        first_ms = 1u;
    }

    prv_boot_led_blink_start();
    if (!prv_start_beacon_rx(first_ms, ND_RX_REASON_BOOT)) {
        prv_schedule_short_boot_retry();
    }
}

static bool prv_restart_current_rx_window(void)
{
    ND_RxReason_t reason = s_rx_reason;
    uint32_t remain_ms = prv_ms_until(s_rx_window_deadline_ms);

    if (remain_ms < 20u) {
        return false;
    }
    if (reason == ND_RX_REASON_NONE) {
        if (s_boot_listen_active) {
            reason = ND_RX_REASON_BOOT;
        } else if (s_sync_state == ND_SYNC_UNSYNC_SEARCH) {
            reason = ND_RX_REASON_SEARCH;
        } else {
            reason = ND_RX_REASON_MAIN;
        }
    }
    return prv_start_beacon_rx(remain_ms, reason);
}

static void prv_continue_boot_listen_or_schedule(void)
{
    if (prv_should_pause_runtime_for_ble()) {
        prv_clear_pending_runtime_rx_events();
        prv_stop_sensor_and_tx_timers();
        return;
    }

    if (s_boot_listen_active) {
        uint32_t remain_ms = prv_ms_until(s_boot_listen_deadline_ms);
        if (remain_ms > 0u) {
            uint32_t next_ms = remain_ms;
            if (next_ms > ND_BOOT_RX_WINDOW_MS) {
                next_ms = ND_BOOT_RX_WINDOW_MS;
            }
            if (next_ms == 0u) {
                next_ms = 1u;
            }
            if (!s_boot_led_blink_active) {
                prv_boot_led_blink_start();
            }
            if (!prv_start_beacon_rx(next_ms, ND_RX_REASON_BOOT)) {
                prv_schedule_short_boot_retry();
            }
            return;
        }

        prv_boot_led_blink_stop();
        s_boot_listen_active = false;
        s_boot_listen_deadline_ms = 0u;
        prv_resume_schedule_after_boot_search();
        prv_request_stop_mode();
        return;
    }

    prv_schedule_beacon_window();
    if (s_runtime_enabled) {
        prv_schedule_reminder_window();
        prv_schedule_sensor_and_tx();
    } else {
        prv_stop_sensor_and_tx_timers();
    }
    prv_request_stop_mode_if_possible();
}

static void prv_schedule_beacon_window(void)
{
    uint64_t now;
    uint32_t interval_sec;
    uint64_t next;
    uint64_t delta_centi;
    uint32_t delta_ms;
    uint32_t early_ms;

    if (!UI_Time_IsValid()) {
        prv_schedule_unsync_search_scan();
        return;
    }

    now = UI_Time_NowCenti2016();

    if (s_sync_state == ND_SYNC_UNSYNC_SEARCH) {
        prv_schedule_unsync_search_scan();
        return;
    }

    if (s_sync_state == ND_SYNC_PHASE_WALK) {
        if (!prv_get_next_phase_walk_scan_centi(now, &next)) {
            prv_enter_unsync_search();
            return;
        }

        delta_centi = (next > now) ? (next - now) : 1u;
        delta_ms = (uint32_t)(delta_centi * 10u);
        if (delta_ms == 0u) {
            delta_ms = 1u;
        }

        (void)UTIL_TIMER_Stop(&s_tmr_beacon_sched);
        (void)UTIL_TIMER_SetPeriod(&s_tmr_beacon_sched, delta_ms);
        (void)UTIL_TIMER_Start(&s_tmr_beacon_sched);
        prv_request_stop_mode_if_possible();
        return;
    }

    interval_sec = prv_get_beacon_interval_sec();
    if (!prv_get_next_predicted_beacon_centi(now, &next)) {
        next = prv_next_event_centi(now, interval_sec, 0u);
    }

    s_force_gw_phase_scan = prv_select_next_gw_phase_scan();
    delta_centi = (next > now) ? (next - now) : 1u;
    delta_ms = (uint32_t)(delta_centi * 10u);
    early_ms = prv_get_beacon_early_wake_ms();
    if (s_force_gw_phase_scan) {
        early_ms += ND_GW_PHASE_SCAN_EXTRA_MS;
    }
    if (delta_ms > early_ms) {
        delta_ms -= early_ms;
    } else {
        delta_ms = 1u;
    }

    (void)UTIL_TIMER_Stop(&s_tmr_beacon_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_beacon_sched, delta_ms);
    (void)UTIL_TIMER_Start(&s_tmr_beacon_sched);
    prv_request_stop_mode_if_possible();
}

static void prv_schedule_reminder_window(void)
{
    (void)UTIL_TIMER_Stop(&s_tmr_reminder_sched);
}

static void prv_schedule_sensor_and_tx(void)
{
    const UI_Config_t *cfg;
    uint8_t node;
    uint64_t now;
    uint32_t period;
    uint32_t sensor_off;
    uint32_t base_tx;
    uint32_t tx_off;
    uint64_t next_sensor;
    uint64_t next_tx;
    uint32_t ds_ms;
    uint32_t dt_ms;

    if (!s_runtime_enabled) {
        prv_stop_sensor_and_tx_timers();
        s_sensor_ready = false;
        return;
    }

    prv_refresh_mode_from_config();
    cfg = UI_GetConfig();
    node = cfg->node_num;

    if (s_test_mode && (node >= UI_TESTMODE_MAX_NODES)) {
        return;
    }
    if (prv_is_two_minute_mode_active() && (node >= 10u)) {
        return;
    }

    now = UI_Time_NowCenti2016();
    period = prv_get_runtime_cycle_sec();
    sensor_off = s_test_mode ? UI_ND_SENSOR_START_S_TEST : UI_ND_SENSOR_START_S_NORMAL;
    base_tx = prv_get_tx_base_offset_sec();
    if (prv_is_two_minute_mode_active()) {
        sensor_off = 6u;
    }

    tx_off = base_tx + (uint32_t)node * 2u;
    next_sensor = prv_next_event_centi(now, period, sensor_off);
    next_tx = prv_next_event_centi(now, period, tx_off);
    ds_ms = (uint32_t)((next_sensor > now) ? ((next_sensor - now) * 10u) : 1u);
    dt_ms = (uint32_t)((next_tx > now) ? ((next_tx - now) * 10u) : 1u);
    dt_ms += prv_get_tx_in_slot_delay_ms(node);

    (void)UTIL_TIMER_Stop(&s_tmr_sensor_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_sensor_sched, ds_ms);
    (void)UTIL_TIMER_Start(&s_tmr_sensor_sched);

    (void)UTIL_TIMER_Stop(&s_tmr_tx_sched);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_tx_sched, dt_ms);
    (void)UTIL_TIMER_Start(&s_tmr_tx_sched);
    prv_request_stop_mode_if_possible();
}

void UI_Hook_OnOpKeyPressed(void)
{
    ND_SensorResult_t r;
    char ts[48];
    char msg[160];
    uint32_t pulse;
    uint16_t sx = UI_NODE_MEAS_UNUSED_U16;
    uint16_t sy = UI_NODE_MEAS_UNUSED_U16;
    uint16_t sz = UI_NODE_MEAS_UNUSED_U16;
    uint16_t sa = UI_NODE_MEAS_UNUSED_U16;

    ND_App_OnBleSessionStart();
    prv_led1(true);
    (void)ND_Sensors_MeasureAll(&r, UI_GetConfig()->sensor_en_mask);
    UI_BLE_EnableForMs(UI_BLE_ACTIVE_MS);
    UI_Time_FormatNow(ts, sizeof(ts));
    pulse = r.pulse_cnt;
    if (prv_sensor_xyz_valid(&r)) {
        sx = prv_encode_payload_x(r.x);
        sy = prv_encode_payload_y(r.y);
        sz = prv_encode_payload_z(r.z);
    }
    if (prv_sensor_adc_valid(&r)) {
        sa = prv_encode_payload_adc(r.adc);
    }
    (void)snprintf(msg, sizeof(msg), "<%s:%u,%u,%u,%u,%lu>\r\n",
                   ts, (unsigned)sx, (unsigned)sy, (unsigned)sz,
                   (unsigned)sa, (unsigned long)pulse);
    UI_UART_SendString(msg);
    if (!UI_BLE_IsActive()) {
        prv_led1(false);
    }
}

bool UI_Hook_OnTestStartRequested(void)
{
    ND_SensorResult_t r;
    bool measure_ok;

    prv_led1(true);
    if (UI_BLE_IsActive()) {
        UI_BLE_EnsureSerialReady();
    }
    prv_start_test_session_from_cmd();
    measure_ok = ND_Sensors_MeasureAll(&r, UI_GetConfig()->sensor_en_mask);
    if (measure_ok) {
        prv_send_test_result_ble(&r);
    }
    if (!UI_BLE_IsActive()) {
        prv_led1(false);
    }
    return measure_ok;
}

bool UI_Hook_OnSyncRequested(void)
{
    if (!s_inited) {
        return false;
    }

    if (UI_BLE_IsActive()) {
        prv_hold_ble_timeout_for_sync();
    } else {
        prv_clear_saved_ble_timeout_for_sync();
    }
    prv_hold_ble_for_sync();
    s_evt_flags |= ND_EVT_SYNC_START;
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
    return true;
}

static bool prv_is_user_abortable_beacon_search_active(void)
{
    if (UI_BLE_IsActive()) {
        return false;
    }

    if (s_boot_listen_active) {
        return true;
    }

    if (s_sync_state == ND_SYNC_UNSYNC_SEARCH) {
        return true;
    }

    if ((s_state == ND_STATE_RX_BEACON) &&
        ((s_rx_reason == ND_RX_REASON_BOOT) ||
         (s_rx_reason == ND_RX_REASON_SEARCH))) {
        return true;
    }

    if (((s_evt_flags & (ND_EVT_BOOT_LISTEN_START | ND_EVT_BEACON_LISTEN_START)) != 0u) &&
        (s_sync_state == ND_SYNC_BOOT_SEARCH)) {
        return true;
    }

    return false;
}

bool ND_App_StopBeaconSearchAndEnterStop(void)
{
    if (!s_inited) {
        return false;
    }

    if (!prv_is_user_abortable_beacon_search_active()) {
        return false;
    }

    s_evt_flags &= ~(ND_EVT_BOOT_LISTEN_START |
                     ND_EVT_BEACON_LISTEN_START |
                     ND_EVT_SENSOR_START |
                     ND_EVT_TX_START |
                     ND_EVT_REMINDER_LISTEN_START |
                     ND_EVT_TX_RECOVER |
                     ND_EVT_SYNC_START |
                     ND_EVT_RADIO_TX_DONE_LED_PULSE |
                     ND_EVT_RADIO_RX_DONE_LED_PULSE |
                     ND_EVT_SYNC_NOTIFY_MASK);

    prv_abort_active_radio_for_ble();
    prv_enter_unsynced_idle();
    prv_request_stop_mode();
    return true;
}

void ND_App_OnBleSessionStart(void)
{
    if (!s_inited) {
        return;
    }

    prv_boot_beacon_leds(false);

    if (s_boot_listen_active) {
        s_boot_listen_stop_after_ble = true;
    }

    prv_boot_led_blink_stop();
    prv_abort_active_radio_for_ble();
    (void)UTIL_TIMER_Stop(&s_tmr_boot_listen);
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    prv_clear_pending_runtime_rx_events();
    prv_stop_sensor_and_tx_timers();
    s_evt_flags &= ~(ND_EVT_BOOT_LISTEN_START |
                     ND_EVT_BEACON_LISTEN_START |
                     ND_EVT_SENSOR_START |
                     ND_EVT_TX_START |
                     ND_EVT_REMINDER_LISTEN_START |
                     ND_EVT_TX_RECOVER |
                     ND_EVT_SYNC_START |
                     ND_EVT_ENTER_STOP |
                     ND_EVT_RADIO_TX_DONE_LED_PULSE |
                     ND_EVT_RADIO_RX_DONE_LED_PULSE |
                     ND_EVT_SYNC_NOTIFY_MASK);
    s_sensor_ready = false;
    s_sync_cmd_active = false;
    s_sync_tx_wait_beacon_pending = false;
    s_ble_runtime_resume_allowed = false;
    prv_clear_saved_ble_timeout_for_sync();
}

void ND_App_OnBleSessionEnd(void)
{
    if (!s_inited) {
        return;
    }

    prv_boot_led_blink_stop();
    prv_abort_active_radio_for_ble();
    (void)UTIL_TIMER_Stop(&s_tmr_boot_listen);
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    prv_clear_pending_runtime_rx_events();
    s_evt_flags &= ~(ND_EVT_BOOT_LISTEN_START |
                     ND_EVT_BEACON_LISTEN_START |
                     ND_EVT_SENSOR_START |
                     ND_EVT_TX_START |
                     ND_EVT_REMINDER_LISTEN_START |
                     ND_EVT_TX_RECOVER |
                     ND_EVT_SYNC_START |
                     ND_EVT_RADIO_TX_DONE_LED_PULSE |
                     ND_EVT_RADIO_RX_DONE_LED_PULSE |
                     ND_EVT_SYNC_NOTIFY_MASK);
    s_sensor_ready = false;
    s_sync_cmd_active = false;
    s_sync_tx_wait_beacon_pending = false;
    s_ble_runtime_resume_allowed = false;
    prv_clear_saved_ble_timeout_for_sync();

    if (s_boot_listen_stop_after_ble) {
        s_boot_listen_stop_after_ble = false;
        prv_enter_unsynced_idle();
        return;
    }

    if (s_sync_state == ND_SYNC_BOOT_SEARCH) {
        prv_resume_schedule_after_boot_search();
        prv_request_stop_mode_if_possible();
        return;
    }

    prv_continue_boot_listen_or_schedule();
    prv_request_stop_mode_if_possible();
}

void ND_App_Process(void)
{
    if (!s_inited) {
        return;
    }

    UI_BLE_Process();

    uint32_t ev = s_evt_flags;

    if ((ev & ND_EVT_RADIO_TX_DONE_LED_PULSE) != 0u) {
        s_evt_flags &= ~ND_EVT_RADIO_TX_DONE_LED_PULSE;
        prv_led1_blocking_pulse_ms(ND_RADIO_LED_PULSE_MS);
        ev = s_evt_flags;
    }

    if ((ev & ND_EVT_RADIO_RX_DONE_LED_PULSE) != 0u) {
        s_evt_flags &= ~ND_EVT_RADIO_RX_DONE_LED_PULSE;
        prv_led1_blocking_pulse_ms(ND_RADIO_LED_PULSE_MS);
        ev = s_evt_flags;
    }

    if ((ev & ND_EVT_SYNC_NOTIFY_MASK) != 0u) {
        if ((ev & ND_EVT_SYNC_TX_NOTIFY) != 0u) {
            s_evt_flags &= ~ND_EVT_SYNC_TX_NOTIFY;
            prv_send_sync_status(ND_SYNC_NOTIFY_TX_STR);
        }
        if ((ev & ND_EVT_SYNC_DONE_NOTIFY) != 0u) {
            s_evt_flags &= ~ND_EVT_SYNC_DONE_NOTIFY;
            prv_send_sync_status(ND_SYNC_NOTIFY_DONE_STR);
            prv_restore_ble_timeout_after_sync_success();
            prv_resume_runtime_after_sync_completion();
        }
        if ((ev & ND_EVT_SYNC_TIMEOUT_NOTIFY) != 0u) {
            s_evt_flags &= ~ND_EVT_SYNC_TIMEOUT_NOTIFY;
            prv_send_sync_status(ND_SYNC_NOTIFY_TIMEOUT_STR);
            prv_clear_saved_ble_timeout_for_sync();
            prv_resume_runtime_after_sync_completion();
        }
        if ((ev & ND_EVT_SYNC_TX_FAIL_NOTIFY) != 0u) {
            s_evt_flags &= ~ND_EVT_SYNC_TX_FAIL_NOTIFY;
            prv_send_sync_status(ND_SYNC_NOTIFY_TX_FAIL_STR);
            prv_clear_saved_ble_timeout_for_sync();
            prv_resume_runtime_after_sync_completion();
        }
        ev = s_evt_flags;
    }

    if ((ev & ND_EVT_TEST_SESSION_EXPIRE) != 0u) {
        s_evt_flags &= ~ND_EVT_TEST_SESSION_EXPIRE;
        prv_stop_test_session();
        prv_reschedule_main_if_pending();
        return;
    }

    if ((ev & ND_EVT_BOOT_LISTEN_START) != 0u) {
        s_evt_flags &= ~ND_EVT_BOOT_LISTEN_START;
        if (!s_boot_listen_active) {
            prv_begin_boot_listen();
        } else {
            prv_continue_boot_listen_or_schedule();
        }
        prv_reschedule_main_if_pending();
        return;
    }

    if ((ev & ND_EVT_BEACON_LISTEN_START) != 0u) {
        s_evt_flags &= ~ND_EVT_BEACON_LISTEN_START;
        bool started = false;
        if (!UI_Time_IsValid()) {
            started = prv_start_beacon_rx(ND_SEARCH_RX_WINDOW_MS, ND_RX_REASON_SEARCH);
        } else if (s_sync_state == ND_SYNC_PHASE_WALK) {
            started = prv_start_beacon_rx(ND_PHASE_WALK_RX_WINDOW_MS, ND_RX_REASON_MAIN);
            if (started) {
                prv_advance_phase_walk_index();
            }
        } else if (s_sync_state == ND_SYNC_UNSYNC_SEARCH) {
            started = prv_start_beacon_rx(ND_SEARCH_RX_WINDOW_MS, ND_RX_REASON_SEARCH);
        } else {
            started = prv_start_beacon_rx(prv_get_beacon_rx_window_ms_with_phase_scan(), ND_RX_REASON_MAIN);
        }
        s_force_gw_phase_scan = false;
        if (!started) {
            prv_schedule_short_beacon_retry();
        }
        prv_reschedule_main_if_pending();
        return;
    }

    if ((ev & ND_EVT_REMINDER_LISTEN_START) != 0u) {
        s_evt_flags &= ~ND_EVT_REMINDER_LISTEN_START;
        prv_reschedule_main_if_pending();
        return;
    }

    if ((ev & ND_EVT_SYNC_START) != 0u) {
        s_evt_flags &= ~ND_EVT_SYNC_START;
        if (!prv_start_sync_request_tx()) {
            prv_send_sync_status(ND_SYNC_NOTIFY_TX_FAIL_STR);
            prv_clear_saved_ble_timeout_for_sync();
            prv_resume_runtime_after_sync_completion();
        }
        prv_reschedule_main_if_pending();
        return;
    }

    if ((ev & ND_EVT_ENTER_STOP) != 0u) {
        s_evt_flags &= ~ND_EVT_ENTER_STOP;
        prv_enter_stop_now_if_possible();
        prv_reschedule_main_if_pending();
        return;
    }

    if ((ev & ND_EVT_TX_RECOVER) != 0u) {
        s_evt_flags &= ~ND_EVT_TX_RECOVER;
        if (s_state == ND_STATE_TX_DATA) {
            prv_force_tx_recovery(false);
        }
        prv_reschedule_main_if_pending();
        return;
    }

    if ((ev & ND_EVT_SENSOR_START) != 0u) {
        s_evt_flags &= ~ND_EVT_SENSOR_START;
        uint32_t now_sec;
        uint32_t period;
        uint32_t sensor_off;
        uint32_t sensor_slot_id;

        if (!s_runtime_enabled) {
            s_sensor_ready = false;
            prv_reschedule_main_if_pending();
            return;
        }

        now_sec = UI_Time_NowSec2016();
        period = prv_get_runtime_cycle_sec();
        sensor_off = s_test_mode ? UI_ND_SENSOR_START_S_TEST : UI_ND_SENSOR_START_S_NORMAL;
        if (prv_is_two_minute_mode_active()) {
            sensor_off = 6u;
        }

        sensor_slot_id = prv_periodic_slot_id_from_epoch_sec(now_sec, period, sensor_off);
        if (sensor_slot_id == s_last_sensor_slot_id) {
            prv_reschedule_main_if_pending();
            return;
        }
        s_last_sensor_slot_id = sensor_slot_id;
        {
            ND_SensorResult_t measured;
            ND_SensorResult_t merged;
            uint8_t cfg_mask = (uint8_t)(UI_GetConfig()->sensor_en_mask & UI_SENSOR_EN_ALL);
            bool measure_ok;

            prv_set_invalid_sensor_result(&measured);
            measure_ok = ND_Sensors_MeasureAll(&measured, cfg_mask);
            if (measure_ok) {
                if (s_last_sensor_valid) {
                    merged = s_last_sensor;
                } else {
                    prv_set_invalid_sensor_result(&merged);
                }
                prv_merge_sensor_result(&merged, &measured, cfg_mask);
                s_last_sensor = merged;
                s_last_sensor_valid = prv_sensor_result_has_any_valid(&s_last_sensor, cfg_mask);
                s_sensor_ready = s_last_sensor_valid;
                if (s_last_sensor_valid) {
                    prv_led1_pulse_10ms();
                }
            }
        }
        prv_reschedule_main_if_pending();
        return;
    }

    if ((ev & ND_EVT_TX_START) != 0u) {
        s_evt_flags &= ~ND_EVT_TX_START;
        const UI_Config_t *cfg;
        UI_NodeData_t nd;
        ND_SensorResult_t tx_sensor;
        ND_SensorResult_t reload_sensor;
        uint64_t now_centi;
        uint64_t due_tx_centi;
        uint32_t now_sec;
        uint32_t period;
        uint32_t tx_off;
        uint32_t tx_slot_id;
        uint32_t freq_anchor_sec;
        bool retry_scheduled;
        bool payload_ready;
        uint8_t cfg_mask;
        uint8_t tx_payload_len = 0u;

        cfg = UI_GetConfig();
        cfg_mask = (uint8_t)(cfg->sensor_en_mask & UI_SENSOR_EN_ALL);
        now_centi = UI_Time_NowCenti2016();
        now_sec = (uint32_t)(now_centi / 100u);
        period = prv_get_runtime_cycle_sec();
        tx_off = prv_get_tx_base_offset_sec() + (uint32_t)cfg->node_num * 2u;

        if (!s_runtime_enabled) {
            prv_reschedule_main_if_pending();
            return;
        }

        if (s_last_sensor_valid) {
            tx_sensor = s_last_sensor;
        } else {
            prv_set_invalid_sensor_result(&tx_sensor);
        }

        if (!s_sensor_ready && !s_last_sensor_valid) {
            prv_set_invalid_sensor_result(&tx_sensor);
        }
        if ((cfg_mask & UI_SENSOR_EN_PULSE) != 0u) {
            tx_sensor.pulse_cnt = UI_GPIO_GetPulseCount();
        } else {
            tx_sensor.pulse_cnt = 0xFFFFFFFFu;
        }

        if (!prv_is_event_due_now(now_centi, period, tx_off, ND_TX_DUE_LATE_GRACE_CENTI, &due_tx_centi)) {
            prv_schedule_tx_event_at(prv_next_event_centi(now_centi, period, tx_off), cfg->node_num);
            prv_reschedule_main_if_pending();
            return;
        }

        now_sec = (uint32_t)(due_tx_centi / 100u);
        tx_slot_id = prv_periodic_slot_id_from_epoch_sec(now_sec, period, tx_off);

        if (s_state == ND_STATE_RX_BEACON) {
            (void)prv_abort_beacon_rx_for_tx();
        }

        if (s_state != ND_STATE_IDLE) {
            retry_scheduled = prv_schedule_short_tx_retry(period, tx_off);
            if (!retry_scheduled) {
                prv_schedule_sensor_and_tx();
            }
            prv_reschedule_main_if_pending();
            return;
        }

        payload_ready = prv_build_verified_node_payload(cfg,
                                                        &tx_sensor,
                                                        s_beacon_cnt,
                                                        s_node_tx_payload,
                                                        &tx_payload_len,
                                                        &nd);
        if (!payload_ready) {
            if (prv_reload_sensor_snapshot_for_tx(cfg_mask, &reload_sensor)) {
                if ((cfg_mask & UI_SENSOR_EN_PULSE) != 0u) {
                    reload_sensor.pulse_cnt = UI_GPIO_GetPulseCount();
                } else {
                    reload_sensor.pulse_cnt = 0xFFFFFFFFu;
                }
                payload_ready = prv_build_verified_node_payload(cfg,
                                                                &reload_sensor,
                                                                s_beacon_cnt,
                                                                s_node_tx_payload,
                                                                &tx_payload_len,
                                                                &nd);
                if (payload_ready) {
                    tx_sensor = reload_sensor;
                }
            }
        }

        if (!payload_ready) {
            retry_scheduled = prv_schedule_short_tx_retry(period, tx_off);
            if (!retry_scheduled) {
                prv_schedule_sensor_and_tx();
            }
            prv_reschedule_main_if_pending();
            return;
        }

        if (!prv_radio_ready_for_tx()) {
            UI_Radio_MarkRecoverNeeded();
            if (Radio.Sleep != NULL) {
                Radio.Sleep();
            }
            retry_scheduled = prv_schedule_short_tx_retry(period, tx_off);
            if (!retry_scheduled) {
                prv_schedule_sensor_and_tx();
            }
            prv_reschedule_main_if_pending();
            return;
        }
        if ((tx_payload_len == 0u) || !UI_Radio_PrepareTx(tx_payload_len)) {
            UI_Radio_MarkRecoverNeeded();
            if (Radio.Sleep != NULL) {
                Radio.Sleep();
            }
            retry_scheduled = prv_schedule_short_tx_retry(period, tx_off);
            if (!retry_scheduled) {
                prv_schedule_sensor_and_tx();
            }
            prv_reschedule_main_if_pending();
            return;
        }

        freq_anchor_sec = prv_get_data_freq_anchor_sec(now_sec);
        prv_suspend_runtime_rx_around_tx();
        UI_LPM_LockStop();
        s_state = ND_STATE_TX_DATA;
        s_tx_inflight_slot_id = tx_slot_id;
        prv_start_tx_watchdog();
        /* GW RX는 같은 cycle/time대에 공통 data frequency(3rd arg = 0u)를 사용한다. */
        Radio.SetChannel(UI_RF_GetDataFreqHz(freq_anchor_sec, period, 0u));
        s_node_tx_payload_len = tx_payload_len;
        Radio.Send(s_node_tx_payload, s_node_tx_payload_len);
        prv_reschedule_main_if_pending();
        return;
    }

    prv_request_stop_mode_if_possible();
}

void ND_App_Init(void)
{
    if (s_inited) {
        return;
    }

    prv_refresh_mode_from_config();

    /* 부팅 직후 beacon miss를 줄이기 위해 blocking LED blink는 제거했다.
     * 시각 피드백은 prv_begin_boot_listen()의 timer-driven blink가 담당한다. */

    UTIL_TIMER_Create(&s_tmr_boot_listen, 0u, UTIL_TIMER_ONESHOT, prv_tmr_boot_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_beacon_sched, 0u, UTIL_TIMER_ONESHOT, prv_tmr_beacon_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_reminder_sched, 0u, UTIL_TIMER_ONESHOT, prv_tmr_reminder_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_sensor_sched, 0u, UTIL_TIMER_ONESHOT, prv_tmr_sensor_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_tx_sched, 0u, UTIL_TIMER_ONESHOT, prv_tmr_tx_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_tx_watchdog, ND_TX_WATCHDOG_MS, UTIL_TIMER_ONESHOT, prv_tmr_tx_watchdog_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_led1_pulse, 10u, UTIL_TIMER_ONESHOT, prv_led1_pulse_off_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_boot_led_blink, ND_BOOT_BEACON_ON_MS, UTIL_TIMER_ONESHOT, prv_tmr_boot_led_blink_cb, NULL);
    UTIL_TIMER_Create(&s_tmr_test_session, ND_TEST_SESSION_MS, UTIL_TIMER_ONESHOT, prv_tmr_test_session_cb, NULL);

    s_state = ND_STATE_IDLE;
    s_beacon_ok = false;
    s_beacon_cnt = 0u;
    s_runtime_enabled = false;
    s_beacon_miss_count = 0u;
    s_last_beacon_anchor_sec = 0u;
    s_sensor_ready = false;
    s_last_sensor_valid = false;
    prv_set_invalid_sensor_result(&s_last_sensor);
    s_last_sensor_slot_id = 0xFFFFFFFFu;
    s_last_tx_slot_id = 0xFFFFFFFFu;
    s_tx_inflight_slot_id = 0xFFFFFFFFu;
    s_test_session_active = false;
    s_test_session_restore_value = 0u;
    s_test_session_restore_unit = 'H';
    s_boot_listen_stop_after_ble = false;
    s_force_gw_phase_scan = false;
    s_gw_phase_scan_cycle_count = 0u;
    s_boot_listen_active = false;
    s_boot_listen_deadline_ms = 0u;
    s_boot_led_blink_active = false;
    s_boot_led_blink_on = false;
    s_rx_window_deadline_ms = 0u;
    s_rx_reason = ND_RX_REASON_NONE;
    s_sync_cmd_active = false;
    s_sync_tx_wait_beacon_pending = false;
    s_ble_runtime_resume_allowed = false;
    s_unsync_backoff_idx = 0u;
    s_inited = true;

    prv_begin_boot_listen();
}

void ND_Radio_OnTxDone(void)
{
    if (s_sync_tx_wait_beacon_pending) {
        prv_stop_tx_watchdog();
        if (Radio.Sleep != NULL) {
            Radio.Sleep();
        }
        s_state = ND_STATE_IDLE;
        s_tx_inflight_slot_id = 0xFFFFFFFFu;
        UI_LPM_UnlockStop();
        s_sync_tx_wait_beacon_pending = false;
        s_evt_flags |= ND_EVT_RADIO_TX_DONE_LED_PULSE;
        prv_hold_ble_for_sync();
        s_evt_flags |= ND_EVT_SYNC_TX_NOTIFY;
        if (!prv_start_beacon_rx(ND_SYNC_CMD_RX_WINDOW_MS, ND_RX_REASON_SYNC)) {
            s_sync_cmd_active = false;
            s_evt_flags |= ND_EVT_SYNC_TX_FAIL_NOTIFY;
            prv_enter_unsync_search();
        }
        UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
        return;
    }
    s_evt_flags |= ND_EVT_RADIO_TX_DONE_LED_PULSE;
    prv_force_tx_recovery(true);
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

void ND_Radio_OnTxTimeout(void)
{
    uint32_t period_sec = 0u;
    uint32_t tx_off_sec = 0u;
    bool retry_scheduled = false;

    prv_stop_tx_watchdog();
    UI_Radio_MarkRecoverNeeded();
    if (Radio.Sleep != NULL) {
        Radio.Sleep();
    }
    s_state = ND_STATE_IDLE;
    s_tx_inflight_slot_id = 0xFFFFFFFFu;
    UI_LPM_UnlockStop();

    if (s_sync_tx_wait_beacon_pending || s_sync_cmd_active) {
        s_sync_tx_wait_beacon_pending = false;
        s_sync_cmd_active = false;
        s_evt_flags |= ND_EVT_SYNC_TX_FAIL_NOTIFY;
        prv_enter_unsync_search();
        UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
        return;
    }

    if (prv_get_tx_retry_params(&period_sec, &tx_off_sec)) {
        retry_scheduled = prv_schedule_short_tx_retry(period_sec, tx_off_sec);
    }
    if (!retry_scheduled) {
        prv_refresh_runtime_timers_after_tx();
    }
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
}

void ND_Radio_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    UI_Beacon_t beacon;
    const UI_Config_t *cfg = UI_GetConfig();
    uint64_t beacon_epoch_centi;

    (void)rssi;
    (void)snr;

    if (s_state != ND_STATE_RX_BEACON) {
        return;
    }

    Radio.Sleep();
    s_state = ND_STATE_IDLE;
    UI_LPM_UnlockStop();

    if (!UI_Pkt_ParseBeacon(payload, size, &beacon)) {
        if (prv_restart_current_rx_window()) {
            return;
        }
        if (s_rx_reason == ND_RX_REASON_SYNC) {
            s_sync_cmd_active = false;
            s_evt_flags |= ND_EVT_SYNC_TIMEOUT_NOTIFY;
            prv_enter_unsync_search();
            UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
            return;
        }
        if (s_rx_reason == ND_RX_REASON_BOOT) {
            prv_continue_boot_listen_or_schedule();
            return;
        }
        if (s_rx_reason == ND_RX_REASON_MAIN) {
            prv_note_main_beacon_miss();
        } else if (s_rx_reason == ND_RX_REASON_SEARCH) {
            prv_advance_unsync_backoff();
        }
        prv_continue_boot_listen_or_schedule();
        return;
    }

    if (memcmp(beacon.net_id, cfg->net_id, UI_NET_ID_LEN) != 0) {
        if (prv_restart_current_rx_window()) {
            return;
        }
        if (s_rx_reason == ND_RX_REASON_SYNC) {
            s_sync_cmd_active = false;
            s_evt_flags |= ND_EVT_SYNC_TIMEOUT_NOTIFY;
            prv_enter_unsync_search();
            UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
            return;
        }
        if (s_rx_reason == ND_RX_REASON_BOOT) {
            prv_continue_boot_listen_or_schedule();
            return;
        }
        if (s_rx_reason == ND_RX_REASON_MAIN) {
            prv_note_main_beacon_miss();
        } else if (s_rx_reason == ND_RX_REASON_SEARCH) {
            prv_advance_unsync_backoff();
        }
        prv_continue_boot_listen_or_schedule();
        return;
    }

    beacon_epoch_centi = ((uint64_t)UI_Time_Epoch2016_FromCalendar(&beacon.dt) * 100u) +
                         (uint64_t)ND_BEACON_RX_TIME_CORR_CENTI;
    s_evt_flags |= ND_EVT_RADIO_RX_DONE_LED_PULSE;
    UI_Time_SetEpochCenti2016(beacon_epoch_centi);
    prv_apply_setting_ascii(beacon.setting_ascii);
    s_beacon_cnt++;
    s_sync_cmd_active = false;
    prv_enter_locked_from_beacon(&beacon);
    if (s_rx_reason == ND_RX_REASON_SYNC) {
        s_evt_flags |= ND_EVT_SYNC_DONE_NOTIFY;
        UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
    }
    prv_continue_boot_listen_or_schedule();
    UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
    if (s_rx_reason == ND_RX_REASON_BOOT) {
        prv_boot_led_blink_stop();
        prv_request_stop_mode();
    }
}

void ND_Radio_OnRxTimeout(void)
{
    if (s_state != ND_STATE_RX_BEACON) {
        return;
    }

    Radio.Sleep();
    s_state = ND_STATE_IDLE;
    UI_LPM_UnlockStop();

    if (s_rx_reason == ND_RX_REASON_SYNC) {
        s_sync_cmd_active = false;
        s_evt_flags |= ND_EVT_SYNC_TIMEOUT_NOTIFY;
        prv_enter_unsync_search();
        UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
        return;
    }
    if (s_rx_reason == ND_RX_REASON_BOOT) {
        prv_continue_boot_listen_or_schedule();
        return;
    }
    if (s_rx_reason == ND_RX_REASON_MAIN) {
        prv_note_main_beacon_miss();
    } else if (s_rx_reason == ND_RX_REASON_SEARCH) {
        s_beacon_ok = false;
        prv_advance_unsync_backoff();
    }
    prv_continue_boot_listen_or_schedule();
}

void ND_Radio_OnRxError(void)
{
    if (s_state != ND_STATE_RX_BEACON) {
        return;
    }

    Radio.Sleep();
    s_state = ND_STATE_IDLE;
    UI_LPM_UnlockStop();

    if (s_rx_reason == ND_RX_REASON_SYNC) {
        s_sync_cmd_active = false;
        s_evt_flags |= ND_EVT_SYNC_TIMEOUT_NOTIFY;
        prv_enter_unsync_search();
        UTIL_SEQ_SetTask(UI_TASK_BIT_ND_MAIN, 0);
        return;
    }
    if (prv_restart_current_rx_window()) {
        return;
    }
    if (s_rx_reason == ND_RX_REASON_BOOT) {
        prv_continue_boot_listen_or_schedule();
        return;
    }
    if (s_rx_reason == ND_RX_REASON_MAIN) {
        prv_note_main_beacon_miss();
    } else if (s_rx_reason == ND_RX_REASON_SEARCH) {
        prv_advance_unsync_backoff();
    }
    prv_continue_boot_listen_or_schedule();
}
