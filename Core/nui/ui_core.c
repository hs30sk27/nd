#include "ui_core.h"
#include "ui_conf.h"
#include "ui_types.h"
#include "ui_uart.h"
#include "ui_gpio.h"
#include "ui_cmd.h"
#include "ui_time.h"
#include "ui_ble.h"
#include "ui_lpm.h"
#include "nd_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "stm32wlxx_hal.h" /* __weak */

#include <string.h>

/* -------------------------------------------------------------------------- */
/* Hook functions (ND/GW에서 override 가능)                                     */
/* -------------------------------------------------------------------------- */
__weak void UI_Hook_OnOpKeyPressed(void) {}

/* -------------------------------------------------------------------------- */
/* UART 명령 수신 파서                                                        */
/*                                                                            */
/* 요구사항(최종):                                                            */
/*   - UART1 명령은 반드시 "<CMD>\r\n" 형태                                  */
/*   - BLE 특성상 notification이 분할되어 들어올 수 있으므로                   */
/*     "100ms 이내로 들어온 바이트"는 같은 메시지로 취급                      */
/*   - 블루투스 시작 시 튀는 쓰레기 데이터에는 아무 응답도 하지 않음           */
/*     (ERROR를 보내지 않음)                                                   */
/*                                                                            */
/* 정책:                                                                      */
/*   - '<'가 오기 전까지는 전부 무시                                           */
/*   - '>' 이후에는 반드시 CRLF가 와야 명령 확정                              */
/*   - 수신이 100ms 이상 끊기면(미완성 프레임) 버리고 리셋                     */
/* -------------------------------------------------------------------------- */
typedef enum
{
    CMD_RX_WAIT_START = 0,
    CMD_RX_IN_FRAME,
    CMD_RX_WAIT_LF,
} CmdRxState_t;

static CmdRxState_t s_cmdrx_state = CMD_RX_WAIT_START;
static char         s_cmdrx_buf[UI_UART_LINE_MAX];
static uint16_t     s_cmdrx_len = 0;

static void prv_cmdrx_reset(void)
{
    s_cmdrx_state = CMD_RX_WAIT_START;
    s_cmdrx_len   = 0;
}

static void prv_cmdrx_finalize(void)
{
    if (s_cmdrx_len >= (UI_UART_LINE_MAX - 1u))
    {
        prv_cmdrx_reset();
        return;
    }

    s_cmdrx_buf[s_cmdrx_len] = '\0';
    if (s_cmdrx_len > 0u)
    {
        /* 여기까지 왔다는 것은 <...>CRLF 프레임이 완성된 경우만 */
        UI_Cmd_ProcessLine(s_cmdrx_buf);
    }
    prv_cmdrx_reset();
}

static void prv_cmdrx_feed(uint8_t b)
{
    switch (s_cmdrx_state)
    {
        case CMD_RX_WAIT_START:
        {
            if (b == '<')
            {
                s_cmdrx_len = 0;
                s_cmdrx_buf[s_cmdrx_len++] = '<';
                s_cmdrx_state = CMD_RX_IN_FRAME;
            }
            break;
        }

        case CMD_RX_IN_FRAME:
        {
            /* 재동기: 프레임 중간에 '<'가 오면 새 프레임으로 간주 */
            if (b == '<')
            {
                s_cmdrx_len = 0;
                s_cmdrx_buf[s_cmdrx_len++] = '<';
                s_cmdrx_state = CMD_RX_IN_FRAME;
                break;
            }

            /* 프레임 내부에서 CR/LF가 나오면 쓰레기로 보고 폐기 */
            if (b == '\r' || b == '\n')
            {
                prv_cmdrx_reset();
                break;
            }

            if (s_cmdrx_len < (UI_UART_LINE_MAX - 1u))
            {
                s_cmdrx_buf[s_cmdrx_len++] = (char)b;
            }
            else
            {
                /* overflow: 폐기 */
                prv_cmdrx_reset();
                break;
            }

            if (b == '>')
            {
                /* 반드시 CRLF(또는 최소 LF)를 기다린다 */
                s_cmdrx_state = CMD_RX_WAIT_LF;
            }
            break;
        }

        case CMD_RX_WAIT_LF:
        {
            if (b == '\r')
            {
                /* CR은 무시하고 LF를 기다림 */
                break;
            }
            if (b == '\n')
            {
                prv_cmdrx_finalize();
                break;
            }
            if (b == '<')
            {
                /* 종결자 없이 다음 프레임 시작 -> 이전 프레임 폐기 */
                s_cmdrx_len = 0;
                s_cmdrx_buf[s_cmdrx_len++] = '<';
                s_cmdrx_state = CMD_RX_IN_FRAME;
                break;
            }

            /* 그 외: 쓰레기로 보고 리셋 */
            prv_cmdrx_reset();
            break;
        }

        default:
            prv_cmdrx_reset();
            break;
    }
}

/* -------------------------------------------------------------------------- */
static void UI_TaskMain(void)
{
    /* 1) UART 명령 파싱(프레임: <CMD>CRLF only) */
    uint8_t b = 0;
    while (UI_UART_ReadByte(&b))
    {
        prv_cmdrx_feed(b);
    }

    /* 100ms 이상 끊기면 미완성 프레임은 폐기(응답 없음) */
    if (s_cmdrx_state != CMD_RX_WAIT_START)
    {
        uint32_t now  = UTIL_TIMER_GetCurrentTime();
        uint32_t last = UI_UART_GetLastRxMs();
        if ((uint32_t)(now - last) >= UI_UART_COALESCE_MS)
        {
            prv_cmdrx_reset();
        }
    }

    /* 2) GPIO 이벤트 처리 (ISR에서 delay 금지이므로 여기서 처리) */
    uint32_t ev = UI_GPIO_FetchEvents();
    bool key_abort_handled = false;

    if ((ev & (UI_GPIO_EVT_TEST_KEY | UI_GPIO_EVT_OP_KEY)) != 0u)
    {
        /* boot/unsync beacon 탐색 중에는 아무 키나 탐색 중지 후 stop mode로 진입 */
        key_abort_handled = ND_App_StopBeaconSearchAndEnterStop();
    }

    if (!key_abort_handled && ((ev & UI_GPIO_EVT_TEST_KEY) != 0u))
    {
        /* 공통: TEST_KEY = BLE ON / timeout 연장 */
        if (!UI_BLE_IsActive())
        {
            ND_App_OnBleSessionStart();
        }
        UI_BLE_EnableForMs(UI_BLE_ACTIVE_MS);
    }

    if (!key_abort_handled && ((ev & UI_GPIO_EVT_OP_KEY) != 0u))
    {
        /* 공통: OP_KEY = BLE OFF */
        if (UI_BLE_IsActive())
        {
            UI_BLE_RequestStopNow();
        }
    }

    /* 3) BLE 이벤트 처리 (task bit이 부족한 경우 UI_MAIN에서 처리됨) */
    UI_BLE_Process();

    /* 4) Role(GW/ND) 이벤트 처리 (task bit이 부족한 경우 UI_MAIN에서 처리됨) */
    /* 4) ND 이벤트 처리 */
    ND_App_Process();
}

void UI_Core_ClearFlagsBeforeStop(void)
{
    /* Stop 진입 전: 미완성 명령 프레임/파서 상태를 초기화 */
    prv_cmdrx_reset();
}

void UI_Init(void)
{

    /* Task 등록 */
    UTIL_SEQ_RegTask(UI_TASK_BIT_UI_MAIN, 0, UI_TaskMain);

    /* 모듈 init */
    (void)UI_GetConfig(); /* config default init */
    UI_Time_Init();
    UI_LPM_Init();
    UI_GPIO_Init();
    UI_UART_Init();
    UI_BLE_Init();

    /* 역할별 앱은 UI에서 직접 초기화한다.
     * - main.c에서 별도 ND_App_Init(); 호출이 빠져도 동작해야 함
     * - 이전 버전에서는 이 초기화가 빠지면 BEACON ON/SETTING 후 타이머 객체가
     *   미생성 상태로 접근되어 비콘 미동작 또는 HardFault 원인이 될 수 있었다.
     */
    ND_App_Init();

    /* 초기 한 번 task 실행 유도(상태 정리) */
    UTIL_SEQ_SetTask(UI_TASK_BIT_UI_MAIN, 0);
}
