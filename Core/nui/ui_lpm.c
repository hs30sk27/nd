#include "ui_lpm.h"
#include "ui_conf.h"

/* Stop 진입 전 런타임 플래그 정리용 */
#include "ui_core.h"
#include "ui_gpio.h"
#include "ui_uart.h"
#include "ui_ble.h"
#include "ui_time.h"
#include "ui_radio.h"
#include "radio.h"

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

static void prv_force_stop_pin_levels(void)
{
#if defined(ICM20948_CS_GPIO_Port) && defined(ICM20948_CS_Pin)
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_SET);
#endif
#if defined(ADC_CS_GPIO_Port) && defined(ADC_CS_Pin)
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
#endif
#if defined(ADC_EN_GPIO_Port) && defined(ADC_EN_Pin)
    HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, GPIO_PIN_RESET);
#endif
#if defined(BT_EN_GPIO_Port) && defined(BT_EN_Pin)
    HAL_GPIO_WritePin(BT_EN_GPIO_Port, BT_EN_Pin, GPIO_PIN_RESET);
#endif
#if defined(LED0_GPIO_Port) && defined(LED0_Pin)
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
#endif
#if defined(LED1_GPIO_Port) && defined(LED1_Pin)
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
#endif
#if defined(RF_TXEN_GPIO_Port) && defined(RF_TXEN_Pin)
    HAL_GPIO_WritePin(RF_TXEN_GPIO_Port, RF_TXEN_Pin, GPIO_PIN_RESET);
#endif
#if defined(RF_RXEN_GPIO_Port) && defined(RF_RXEN_Pin)
    HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_RESET);
#endif
}

static void prv_set_gpio_analog(GPIO_TypeDef *port, uint32_t pins)
{
    GPIO_InitTypeDef g = {0};

    if ((port == NULL) || (pins == 0u))
    {
        return;
    }

    g.Pin = pins;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &g);
}

static void prv_restore_gpio_input(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef g = {0};

    if ((port == NULL) || (pin == 0u))
    {
        return;
    }

    g.Pin = pin;
    g.Mode = mode;
    g.Pull = pull;
    HAL_GPIO_Init(port, &g);
}

static void prv_abort_peripheral_activity_before_deinit(void)
{
#if defined(HAL_UART_MODULE_ENABLED)
    (void)HAL_UART_Abort(&huart1);
#endif

#if defined(HAL_SPI_MODULE_ENABLED)
    (void)HAL_SPI_Abort(&hspi1);
#endif
}

static void prv_configure_deinited_pins_for_stop(void)
{
#if defined(__HAL_RCC_GPIOA_CLK_ENABLE)
    __HAL_RCC_GPIOA_CLK_ENABLE();
#endif
#if defined(__HAL_RCC_GPIOB_CLK_ENABLE)
    __HAL_RCC_GPIOB_CLK_ENABLE();
#endif

//#if defined(BATT_LVL_GPIO_Port) && defined(BATT_LVL_Pin)
//    prv_set_gpio_analog(BATT_LVL_GPIO_Port, BATT_LVL_Pin);
//#endif
#if defined(ICM20948_INT_GPIO_Port) && defined(ICM20948_INT_Pin)
    prv_set_gpio_analog(ICM20948_INT_GPIO_Port, ICM20948_INT_Pin);
#endif
//#if defined(PULSE_IN_GPIO_Port) && defined(PULSE_IN_Pin)
//    prv_set_gpio_analog(PULSE_IN_GPIO_Port, PULSE_IN_Pin);
//#endif
#if defined(BLE_TX_GPIO_Port) && defined(BLE_TX_Pin)
    prv_set_gpio_analog(BLE_TX_GPIO_Port, BLE_TX_Pin);
#endif
#if defined(BLE_RX_GPIO_Port) && defined(BLE_RX_Pin)
    prv_set_gpio_analog(BLE_RX_GPIO_Port, BLE_RX_Pin);
#endif
#if defined(GPIOB)
    prv_set_gpio_analog(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
#endif
}

static void prv_restore_pins_after_stop(void)
{
#if defined(__HAL_RCC_GPIOA_CLK_ENABLE)
    __HAL_RCC_GPIOA_CLK_ENABLE();
#endif

#if defined(BATT_LVL_GPIO_Port) && defined(BATT_LVL_Pin)
    prv_restore_gpio_input(BATT_LVL_GPIO_Port, BATT_LVL_Pin, GPIO_MODE_INPUT, GPIO_PULLUP);
#endif
#if defined(ICM20948_INT_GPIO_Port) && defined(ICM20948_INT_Pin)
    prv_restore_gpio_input(ICM20948_INT_GPIO_Port, ICM20948_INT_Pin, GPIO_MODE_IT_RISING, GPIO_PULLUP);
#endif
#if defined(PULSE_IN_GPIO_Port) && defined(PULSE_IN_Pin)
    prv_restore_gpio_input(PULSE_IN_GPIO_Port, PULSE_IN_Pin, GPIO_MODE_IT_FALLING, GPIO_NOPULL);
#endif
}

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
    /*
     * 주기 센서/비콘/TX 스케줄은 RTC alarm + ND 작업 이벤트로 깨어난 뒤 즉시 이어져야 한다.
     * Stop 직전마다 UART/SPI/ADC를 전부 DeInit 하고 SPI GPIO를 analog 로 바꾸면
     * wake 직후 경로가 너무 공격적으로 바뀌어 주기 작업이 흔들릴 수 있으므로,
     * ND 경로에서는 외부 부하만 낮추고 MCU 내부 런타임 경로는 유지한다.
     */
    UI_Time_SaveToBackupNow();

    if (Radio.Sleep != NULL)
    {
        Radio.Sleep();
    }
    UI_Radio_MarkRecoverNeeded();
    prv_force_stop_pin_levels();

    /* Stop 직전 소프트 상태만 정리하고, 주기 작업에 필요한 공용 peripheral 경로는 유지 */
    UI_Core_ClearFlagsBeforeStop();
    UI_GPIO_ClearEvents();
    UI_UART_ResetRxBuffer();
}

void UI_LPM_AfterStop_ReInitPeripherals(void)
{
    /*
     * Wake-up 직후에는 입력 GPIO만 복구하고,
     * 다음 RX/TX 준비에서 radio config 가 다시 clean 하게 걸리도록 mark 한다.
     */
    prv_restore_pins_after_stop();
    UI_Radio_MarkRecoverNeeded();
}

void UI_LPM_EnterStopNow(void)
{
    /* 동작 중이면 stop 진입 금지 */
    if (UI_LPM_IsStopLocked())
    {
        return;
    }

    /* 실제 STOP 진입/복귀 정리는 stm32_lpm_if.c의 PowerDriver 콜백에서 수행한다. */
//    HAL_SuspendTick();
    UTIL_LPM_EnterLowPower();
//    HAL_ResumeTick();
}
