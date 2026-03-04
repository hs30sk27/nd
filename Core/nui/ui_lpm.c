#include "ui_lpm.h"
#include "ui_conf.h"

/* Stop 진입 전 런타임 플래그 정리용 */
#include "ui_core.h"
#include "ui_gpio.h"
#include "ui_uart.h"
#include "ui_ble.h"
#include "ui_time.h"
#include "ui_fault.h"

#include "stm32_lpm.h"
#include "utilities_def.h" /* CFG_LPM_APPLI_Id */
#include "main.h"

#include "stm32wlxx_hal.h"
#include <stddef.h>

#if defined(HAL_SPI_MODULE_ENABLED)
#include "stm32wlxx_hal_spi.h"
#endif

#if defined(HAL_UART_MODULE_ENABLED)
#include "stm32wlxx_hal_uart.h"
#endif

#if defined(HAL_ADC_MODULE_ENABLED)
#include "stm32wlxx_hal_adc.h"
#endif

/* 주변장치 핸들(프로젝트 main.c에 존재) */
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef  hadc;

#if defined(DMA1_Channel2)
extern DMA_HandleTypeDef  hdma_usart1_tx;
#endif

static void prv_disable_spi_clock(const SPI_HandleTypeDef *hspi)
{
#if defined(SPI1) && defined(__HAL_RCC_SPI1_CLK_DISABLE)
    if ((hspi != NULL) && (hspi->Instance == SPI1))
    {
        __HAL_RCC_SPI1_CLK_DISABLE();
        return;
    }
#endif
#if defined(SPI2) && defined(__HAL_RCC_SPI2_CLK_DISABLE)
    if ((hspi != NULL) && (hspi->Instance == SPI2))
    {
        __HAL_RCC_SPI2_CLK_DISABLE();
        return;
    }
#endif
#if defined(SPI3) && defined(__HAL_RCC_SPI3_CLK_DISABLE)
    if ((hspi != NULL) && (hspi->Instance == SPI3))
    {
        __HAL_RCC_SPI3_CLK_DISABLE();
        return;
    }
#endif
}

static void prv_disable_uart_clock(const UART_HandleTypeDef *huart)
{
#if defined(USART1) && defined(__HAL_RCC_USART1_CLK_DISABLE)
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        return;
    }
#endif
#if defined(USART2) && defined(__HAL_RCC_USART2_CLK_DISABLE)
    if ((huart != NULL) && (huart->Instance == USART2))
    {
        __HAL_RCC_USART2_CLK_DISABLE();
        return;
    }
#endif
#if defined(LPUART1) && defined(__HAL_RCC_LPUART1_CLK_DISABLE)
    if ((huart != NULL) && (huart->Instance == LPUART1))
    {
        __HAL_RCC_LPUART1_CLK_DISABLE();
        return;
    }
#endif
}

static void prv_disable_adc_clock(const ADC_HandleTypeDef *hadc_ptr)
{
#if defined(HAL_ADC_MODULE_ENABLED)
# if defined(ADC) && defined(__HAL_RCC_ADC_CLK_DISABLE)
    if ((hadc_ptr != NULL) && (hadc_ptr->Instance == ADC))
    {
        __HAL_RCC_ADC_CLK_DISABLE();
        return;
    }
# endif
# if defined(ADC1) && defined(__HAL_RCC_ADC_CLK_DISABLE)
    if ((hadc_ptr != NULL) && (hadc_ptr->Instance == ADC1))
    {
        __HAL_RCC_ADC_CLK_DISABLE();
        return;
    }
# endif
#endif
}

static volatile uint32_t s_stop_lock = 0;

void UI_LPM_Init(void)
{
    /* 요구사항: off-mode(standby) 비활성화 */
    UTIL_LPM_SetOffMode((1U << CFG_LPM_APPLI_Id), UTIL_LPM_DISABLE);
}

void UI_LPM_LockStop(void)
{
    s_stop_lock++;
    UTIL_LPM_SetStopMode((1U << CFG_LPM_APPLI_Id), UTIL_LPM_DISABLE);
}

void UI_LPM_UnlockStop(void)
{
    if (s_stop_lock > 0u)
    {
        s_stop_lock--;
    }

    if (s_stop_lock == 0u)
    {
        UTIL_LPM_SetStopMode((1U << CFG_LPM_APPLI_Id), UTIL_LPM_ENABLE);
    }
}

bool UI_LPM_IsStopLocked(void)
{
    return (s_stop_lock != 0u);
}

void UI_UART1_TxDma_DeInit(void)
{
#if defined(DMA1_Channel2)
    /*
     * 요구사항:
     *  - UART1 TX DMA는 사용하지 않음.
     *  - 불필요한 DMA 동작/전류 증가를 줄이기 위해 DeInit.
     */
    (void)HAL_DMA_DeInit(&hdma_usart1_tx);
#endif
}

void UI_LPM_BeforeStop_DeInitPeripherals(void)
{
    UI_Time_SaveToBackupNow();
    UI_FAULT_CP(UI_CP_STOP_SAVE, "UI_STOP_SAV", 0u, 0u);

    /*
     * 요구사항: stop 들어가기 전에 spi, uart1, lpuart1, adc deinit
     *
     * NOTE
     *  - Radio(SubGHz)는 별도 파워/상태 관리가 필요할 수 있어 여기서는 건드리지 않습니다.
     *    (LoRa 동작 종료 시점에 Radio.Sleep()을 호출하는 방식으로 전류를 줄이세요.)
     */

    /* ------------------------------------------------------------------ */
    /* SW 상태 정리: 다음 wake-up 후 재진입 시 꼬임 방지                  */
    /* ------------------------------------------------------------------ */
    UI_Core_ClearFlagsBeforeStop();
    UI_GPIO_ClearEvents();
    UI_UART_ResetRxBuffer();
    UI_BLE_ClearFlagsBeforeStop();

#if defined(HAL_ADC_MODULE_ENABLED)
    (void)HAL_ADC_DeInit(&hadc);
    prv_disable_adc_clock(&hadc);
#endif

#if defined(HAL_UART_MODULE_ENABLED)
    (void)HAL_UART_DeInit(&huart1);
    prv_disable_uart_clock(&huart1);
#endif

#if defined(HAL_SPI_MODULE_ENABLED)
    (void)HAL_SPI_DeInit(&hspi1);
    prv_disable_spi_clock(&hspi1);
#endif
}

void UI_LPM_AfterStop_ReInitPeripherals(void)
{
    /*
     * 저전력(배터리) 정책:
     *  - Wake-up 직후 주변장치를 "무조건" ReInit 하지 않습니다.
     *  - 각 모듈이 필요할 때만 Init(Ensure) 하세요.
     *
     * 예)
     *  - BLE ON 시점: UI_UART_EnsureStarted() 호출
     *  - 센서 측정 시점: MX_ADC_Init(), MX_SPI1_Init() 등 필요 부분만 호출
     */
}

void UI_LPM_EnterStopNow(void)
{
    /* 동작 중이면 stop 진입 금지 */
    if (UI_LPM_IsStopLocked())
    {
        return;
    }

    UI_LPM_BeforeStop_DeInitPeripherals();
    UTIL_LPM_EnterLowPower();

    /* wakeup 후: 기본은 아무 것도 하지 않음(최소 전류) */
    UI_LPM_AfterStop_ReInitPeripherals();
}
