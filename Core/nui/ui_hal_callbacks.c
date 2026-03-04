/*
 * ui_hal_callbacks.c
 *
 * NUI 전용 HAL weak callback override.
 *
 * HAL_UART_RxCpltCallback()는 프로젝트의 uart_if.c에서 관리합니다.
 * 여기서는 UART Error callback과 EXTI만 오버라이드합니다.
 */

#include "ui_conf.h"
#include "ui_gpio.h"
#include "ui_uart.h"
#include "stm32wlxx_hal.h"

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UI_UART_ErrorCallback(huart);
}

#if (UI_OVERRIDE_HAL_GPIO_EXTI_CALLBACK == 1u)

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    UI_GPIO_ExtiCallback(GPIO_Pin);
}

#endif /* UI_OVERRIDE_HAL_GPIO_EXTI_CALLBACK */
