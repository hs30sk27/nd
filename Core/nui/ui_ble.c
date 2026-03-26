#include "ui_ble.h"
#include "ui_conf.h"
#include "ui_lpm.h"
#include "ui_uart.h"
#include "nd_app.h"
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 내부 상태 */
/* -------------------------------------------------------------------------- */
static volatile uint32_t s_evt_flags = 0;
#define BLE_EVT_TIMEOUT   (1u << 0)
#define BLE_EVT_STOP_REQ  (1u << 1)
#define BLE_EVT_UART_INIT (1u << 2)
#define BLE_EVT_NAME_APPLY (1u << 3)

static bool s_ble_active = false;
static bool s_led_on = false;
static bool s_uart_init_pending = false;
static bool s_ble_name_apply_pending = false;
static uint32_t s_uart_ready_deadline_ms = 0;
static uint32_t s_bt_on_tick_ms = 0;

static UTIL_TIMER_Object_t s_tmr_timeout;
static UTIL_TIMER_Object_t s_tmr_led;
static UTIL_TIMER_Object_t s_tmr_uart_init;

static char s_ble_name_cmd_buf[48];
static char s_ble_name_cmd_alt_buf[48];
static char s_ble_name_pending_buf[48];

#define UI_BLE_NAMECFG_PWR_OFF_MS      300u
#define UI_BLE_NAMECFG_BOOT_SETTLE_MS 1200u
#define UI_BLE_NAMECFG_CMD_GAP_MS      120u
#define UI_BLE_NAMECFG_RESTART_OFF_MS 300u

static bool prv_apply_device_name_now(const char* name_ascii)
{
#if UI_HAVE_BT_EN
    if ((name_ascii == NULL) || (*name_ascii == '\0')) {
        return false;
    }

    /*
     * 요구 순서:
     *   BLE OFF -> delay -> BLE ON -> delay -> 이름 변경 -> delay -> BLE OFF -> delay -> BLE ON
     */
    UI_BLE_Disable();
    HAL_Delay(UI_BLE_NAMECFG_PWR_OFF_MS);

    UI_BLE_EnableForMs(UI_BLE_ACTIVE_MS);
    UI_BLE_EnsureSerialReady();
    HAL_Delay(UI_BLE_NAMECFG_BOOT_SETTLE_MS);

    UI_UART_SendString("AT\r\n");
    HAL_Delay(UI_BLE_NAMECFG_CMD_GAP_MS);
    UI_UART_SendString("AT\r\n");
    HAL_Delay(UI_BLE_NAMECFG_CMD_GAP_MS);

    (void)snprintf(s_ble_name_cmd_buf, sizeof(s_ble_name_cmd_buf), "AT+NAME%s\r\n", name_ascii);
    UI_UART_SendString(s_ble_name_cmd_buf);
    HAL_Delay(UI_BLE_NAMECFG_CMD_GAP_MS);

    (void)snprintf(s_ble_name_cmd_alt_buf, sizeof(s_ble_name_cmd_alt_buf), "AT+NAME=%s\r\n", name_ascii);
    UI_UART_SendString(s_ble_name_cmd_alt_buf);
    HAL_Delay(UI_BLE_NAMECFG_CMD_GAP_MS);

    UI_BLE_Disable();
    HAL_Delay(UI_BLE_NAMECFG_RESTART_OFF_MS);

    UI_BLE_EnableForMs(UI_BLE_ACTIVE_MS);
    return true;
#else
    (void)name_ascii;
    return false;
#endif
}

static void prv_hw_set_bt(bool on)
{
#if UI_HAVE_BT_EN
    HAL_GPIO_WritePin(BT_EN_GPIO_Port, BT_EN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}

static void prv_hw_set_led0(bool on)
{
#if UI_HAVE_LED0
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}

static void prv_serial_prepare_for_ble_on(void)
{
    /* BT_EN 상승 직전 UART1 pin 구동을 해제해서 리셋 상승 구간 간섭을 줄인다. */
    UI_UART_DeInitLowPower();
}

static void prv_wait_uart_guard_after_bt_on(void)
{
    if (s_bt_on_tick_ms == 0u) {
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_bt_on_tick_ms;
    if (elapsed < UI_BLE_UART_INIT_DELAY_MS) {
        HAL_Delay(UI_BLE_UART_INIT_DELAY_MS - elapsed);
    }
}

static void prv_serial_init_after_ble_delay(void)
{
    /* timer가 일찍 깨우거나, 다른 경로가 먼저 들어와도 BT_EN ON 후 guard는 반드시 보장한다. */
    prv_wait_uart_guard_after_bt_on();
    UI_UART_ReInit();
    s_uart_init_pending = false;
    s_uart_ready_deadline_ms = 0u;
#if (UI_BLE_AT_CMD_AFTER_UART_INIT == 1u)
    HAL_Delay(UI_BLE_AT_CMD_DELAY_MS);
    if (s_ble_active) {
        UI_UART_SendString(UI_BLE_AT_CMD);
    }
#endif
}

static void prv_led_schedule_next(void)
{
    uint32_t next_ms = s_led_on ? UI_LED0_ON_MS : UI_LED0_OFF_MS;
    (void)UTIL_TIMER_Stop(&s_tmr_led);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_led, next_ms);
    (void)UTIL_TIMER_Start(&s_tmr_led);
}

static void prv_tmr_timeout_cb(void *context)
{
    (void)context;
    s_evt_flags |= BLE_EVT_TIMEOUT;
    UTIL_SEQ_SetTask(UI_TASK_BIT_BLE, 0);
}

static void prv_tmr_led_cb(void *context)
{
    (void)context;

    /* LED blink는 task 처리 여부와 무관하게 타이머에서 직접 토글해서 확실히 보이게 한다. */
    if (!s_ble_active) {
        s_led_on = false;
        prv_hw_set_led0(false);
        (void)UTIL_TIMER_Stop(&s_tmr_led);
        return;
    }

    s_led_on = !s_led_on;
    prv_hw_set_led0(s_led_on);
    prv_led_schedule_next();
}

static void prv_tmr_uart_init_cb(void *context)
{
    (void)context;
    s_evt_flags |= BLE_EVT_UART_INIT;
    UTIL_SEQ_SetTask(UI_TASK_BIT_BLE, 0);
}

void UI_BLE_Process(void)
{
    uint32_t ev = s_evt_flags;

    if (ev == 0u) {
        return;
    }

    s_evt_flags = 0u;

    if ((ev & BLE_EVT_STOP_REQ) != 0u) {
        UI_BLE_Disable();
        ND_App_OnBleSessionEnd();
        UI_LPM_EnterStopNow();
        return;
    }

    if ((ev & BLE_EVT_TIMEOUT) != 0u) {
        UI_BLE_Disable();
        ND_App_OnBleSessionEnd();
        UI_LPM_EnterStopNow();
        return;
    }

    if ((ev & BLE_EVT_UART_INIT) != 0u) {
        if (s_ble_active && s_uart_init_pending) {
            prv_serial_init_after_ble_delay();
        }
    }

    if ((ev & BLE_EVT_NAME_APPLY) != 0u) {
        if (s_ble_name_apply_pending) {
            char name_buf[sizeof(s_ble_name_pending_buf)];
            (void)snprintf(name_buf, sizeof(name_buf), "%s", s_ble_name_pending_buf);
            s_ble_name_apply_pending = false;
            s_ble_name_pending_buf[0] = '\0';
            (void)prv_apply_device_name_now(name_buf);
        }
    }
}

static void UI_BLE_Task(void)
{
    UI_BLE_Process();
}

void UI_BLE_Init(void)
{
#if (UI_USE_SEQ_MULTI_TASKS == 1u)
    UTIL_SEQ_RegTask(UI_TASK_BIT_BLE, 0, UI_BLE_Task);
#endif

    (void)UTIL_TIMER_Create(&s_tmr_timeout, UI_BLE_ACTIVE_MS, UTIL_TIMER_ONESHOT, prv_tmr_timeout_cb, NULL);
    (void)UTIL_TIMER_Create(&s_tmr_led, UI_LED0_OFF_MS, UTIL_TIMER_ONESHOT, prv_tmr_led_cb, NULL);
    (void)UTIL_TIMER_Create(&s_tmr_uart_init, UI_BLE_UART_INIT_DELAY_MS, UTIL_TIMER_ONESHOT, prv_tmr_uart_init_cb, NULL);

#if (UI_UART_DEINIT_WHEN_BLE_OFF == 1u)
    UI_UART_DeInitLowPower();
#endif

    s_ble_active = false;
    s_led_on = false;
    s_uart_init_pending = false;
    s_ble_name_apply_pending = false;
    s_uart_ready_deadline_ms = 0u;
    s_bt_on_tick_ms = 0u;
    s_evt_flags = 0u;
    s_ble_name_pending_buf[0] = '\0';
    prv_hw_set_bt(false);
    prv_hw_set_led0(false);
}

void UI_BLE_EnableForMs(uint32_t duration_ms)
{
#if UI_HAVE_BT_EN
    if (duration_ms == 0u) {
        duration_ms = UI_BLE_ACTIVE_MS;
    }

    if (!s_ble_active) {
        s_ble_active = true;
        UI_LPM_LockStop();
        prv_serial_prepare_for_ble_on();
        prv_hw_set_bt(true);
        s_bt_on_tick_ms = HAL_GetTick();
        s_uart_init_pending = true;
        s_uart_ready_deadline_ms = UTIL_TIMER_GetCurrentTime() + UI_BLE_UART_INIT_DELAY_MS;

        (void)UTIL_TIMER_Stop(&s_tmr_uart_init);
        (void)UTIL_TIMER_SetPeriod(&s_tmr_uart_init, UI_BLE_UART_INIT_DELAY_MS);
        (void)UTIL_TIMER_Start(&s_tmr_uart_init);
    }

    /* LED blink는 재호출 때도 항상 다시 arm해서 10/490ms 패턴이 끊기지 않게 한다. */
    s_led_on = true;
    prv_hw_set_led0(true);
    prv_led_schedule_next();

    (void)UTIL_TIMER_Stop(&s_tmr_timeout);
    (void)UTIL_TIMER_SetPeriod(&s_tmr_timeout, duration_ms);
    (void)UTIL_TIMER_Start(&s_tmr_timeout);
#else
    (void)duration_ms;
#endif
}

bool UI_BLE_ApplyDeviceName(const char* name_ascii)
{
#if UI_HAVE_BT_EN
    if ((name_ascii == NULL) || (*name_ascii == '\0')) {
        return false;
    }

    /*
     * 이름 변경은 현재 수신 중인 BLE 프레임 처리와 충돌하지 않게
     * BLE task로 defer해서 수행한다.
     */
    (void)snprintf(s_ble_name_pending_buf, sizeof(s_ble_name_pending_buf), "%s", name_ascii);
    s_ble_name_pending_buf[sizeof(s_ble_name_pending_buf) - 1u] = '\0';
    s_ble_name_apply_pending = true;
    s_evt_flags |= BLE_EVT_NAME_APPLY;
    UTIL_SEQ_SetTask(UI_TASK_BIT_BLE, 0);
    return true;
#else
    (void)name_ascii;
    return false;
#endif
}


void UI_BLE_ExtendMs(uint32_t duration_ms)
{
    if (!s_ble_active) {
        return;
    }
    UI_BLE_EnableForMs(duration_ms);
}

bool UI_BLE_GetRemainingMs(uint32_t* remaining_ms)
{
    uint32_t remain_ms = 0u;

    if (remaining_ms == NULL) {
        return false;
    }

    *remaining_ms = 0u;

    if (!s_ble_active) {
        return false;
    }

    if (UTIL_TIMER_IsRunning(&s_tmr_timeout) == 0u) {
        return false;
    }

    if (UTIL_TIMER_GetRemainingTime(&s_tmr_timeout, &remain_ms) != UTIL_TIMER_OK) {
        return false;
    }

    *remaining_ms = remain_ms;
    return true;
}

void UI_BLE_Disable(void)
{
    if (!s_ble_active) {
        return;
    }

    (void)UTIL_TIMER_Stop(&s_tmr_timeout);
    (void)UTIL_TIMER_Stop(&s_tmr_led);
    (void)UTIL_TIMER_Stop(&s_tmr_uart_init);

    prv_hw_set_led0(false);
    prv_hw_set_bt(false);

#if (UI_UART_DEINIT_WHEN_BLE_OFF == 1u)
    UI_UART_DeInitLowPower();
#endif

    s_ble_active = false;
    s_led_on = false;
    s_uart_init_pending = false;
    s_uart_ready_deadline_ms = 0u;
    s_bt_on_tick_ms = 0u;

    UI_LPM_UnlockStop();
}

bool UI_BLE_IsActive(void)
{
    return s_ble_active;
}

void UI_BLE_RequestStopNow(void)
{
    s_evt_flags |= BLE_EVT_STOP_REQ;
    UTIL_SEQ_SetTask(UI_TASK_BIT_BLE, 0);
}

void UI_BLE_ClearFlagsBeforeStop(void)
{
    (void)UTIL_TIMER_Stop(&s_tmr_timeout);
    (void)UTIL_TIMER_Stop(&s_tmr_led);
    (void)UTIL_TIMER_Stop(&s_tmr_uart_init);
    s_evt_flags = 0u;
    s_ble_active = false;
    s_led_on = false;
    s_uart_init_pending = false;
    s_ble_name_apply_pending = false;
    s_uart_ready_deadline_ms = 0u;
    s_bt_on_tick_ms = 0u;
    s_ble_name_pending_buf[0] = '\0';
    prv_hw_set_led0(false);
    prv_hw_set_bt(false);
#if (UI_UART_DEINIT_WHEN_BLE_OFF == 1u)
    UI_UART_DeInitLowPower();
#endif
}

bool UI_BLE_IsSerialReady(void)
{
    return (!s_uart_init_pending);
}

void UI_BLE_EnsureSerialReady(void)
{
    if (!s_ble_active || !s_uart_init_pending) {
        return;
    }

    uint32_t now = UTIL_TIMER_GetCurrentTime();
    if ((int32_t)(s_uart_ready_deadline_ms - now) > 0) {
        HAL_Delay((uint32_t)(s_uart_ready_deadline_ms - now));
    }

    (void)UTIL_TIMER_Stop(&s_tmr_uart_init);
    if (s_ble_active && s_uart_init_pending) {
        prv_serial_init_after_ble_delay();
    }
}

void UI_Hook_OnBleEndRequested(void)
{
    UI_BLE_RequestStopNow();
}
