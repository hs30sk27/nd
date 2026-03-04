################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/nui/nd_app.c \
../Core/nui/nd_sensors.c \
../Core/nui/ui_ble.c \
../Core/nui/ui_cmd.c \
../Core/nui/ui_config.c \
../Core/nui/ui_core.c \
../Core/nui/ui_crc16.c \
../Core/nui/ui_fault.c \
../Core/nui/ui_gpio.c \
../Core/nui/ui_hal_callbacks.c \
../Core/nui/ui_lpm.c \
../Core/nui/ui_packets.c \
../Core/nui/ui_radio.c \
../Core/nui/ui_rf_plan_kr920.c \
../Core/nui/ui_ringbuf.c \
../Core/nui/ui_time.c \
../Core/nui/ui_uart.c 

OBJS += \
./Core/nui/nd_app.o \
./Core/nui/nd_sensors.o \
./Core/nui/ui_ble.o \
./Core/nui/ui_cmd.o \
./Core/nui/ui_config.o \
./Core/nui/ui_core.o \
./Core/nui/ui_crc16.o \
./Core/nui/ui_fault.o \
./Core/nui/ui_gpio.o \
./Core/nui/ui_hal_callbacks.o \
./Core/nui/ui_lpm.o \
./Core/nui/ui_packets.o \
./Core/nui/ui_radio.o \
./Core/nui/ui_rf_plan_kr920.o \
./Core/nui/ui_ringbuf.o \
./Core/nui/ui_time.o \
./Core/nui/ui_uart.o 

C_DEPS += \
./Core/nui/nd_app.d \
./Core/nui/nd_sensors.d \
./Core/nui/ui_ble.d \
./Core/nui/ui_cmd.d \
./Core/nui/ui_config.d \
./Core/nui/ui_core.d \
./Core/nui/ui_crc16.d \
./Core/nui/ui_fault.d \
./Core/nui/ui_gpio.d \
./Core/nui/ui_hal_callbacks.d \
./Core/nui/ui_lpm.d \
./Core/nui/ui_packets.d \
./Core/nui/ui_radio.d \
./Core/nui/ui_rf_plan_kr920.d \
./Core/nui/ui_ringbuf.d \
./Core/nui/ui_time.d \
./Core/nui/ui_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/nui/%.o Core/nui/%.su Core/nui/%.cyclo: ../Core/nui/%.c Core/nui/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../SubGHz_Phy/App -I../SubGHz_Phy/Target -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Drivers/CMSIS/Include -I"D:/work26/nd/Core/nui" -Oz -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-nui

clean-Core-2f-nui:
	-$(RM) ./Core/nui/nd_app.cyclo ./Core/nui/nd_app.d ./Core/nui/nd_app.o ./Core/nui/nd_app.su ./Core/nui/nd_sensors.cyclo ./Core/nui/nd_sensors.d ./Core/nui/nd_sensors.o ./Core/nui/nd_sensors.su ./Core/nui/ui_ble.cyclo ./Core/nui/ui_ble.d ./Core/nui/ui_ble.o ./Core/nui/ui_ble.su ./Core/nui/ui_cmd.cyclo ./Core/nui/ui_cmd.d ./Core/nui/ui_cmd.o ./Core/nui/ui_cmd.su ./Core/nui/ui_config.cyclo ./Core/nui/ui_config.d ./Core/nui/ui_config.o ./Core/nui/ui_config.su ./Core/nui/ui_core.cyclo ./Core/nui/ui_core.d ./Core/nui/ui_core.o ./Core/nui/ui_core.su ./Core/nui/ui_crc16.cyclo ./Core/nui/ui_crc16.d ./Core/nui/ui_crc16.o ./Core/nui/ui_crc16.su ./Core/nui/ui_fault.cyclo ./Core/nui/ui_fault.d ./Core/nui/ui_fault.o ./Core/nui/ui_fault.su ./Core/nui/ui_gpio.cyclo ./Core/nui/ui_gpio.d ./Core/nui/ui_gpio.o ./Core/nui/ui_gpio.su ./Core/nui/ui_hal_callbacks.cyclo ./Core/nui/ui_hal_callbacks.d ./Core/nui/ui_hal_callbacks.o ./Core/nui/ui_hal_callbacks.su ./Core/nui/ui_lpm.cyclo ./Core/nui/ui_lpm.d ./Core/nui/ui_lpm.o ./Core/nui/ui_lpm.su ./Core/nui/ui_packets.cyclo ./Core/nui/ui_packets.d ./Core/nui/ui_packets.o ./Core/nui/ui_packets.su ./Core/nui/ui_radio.cyclo ./Core/nui/ui_radio.d ./Core/nui/ui_radio.o ./Core/nui/ui_radio.su ./Core/nui/ui_rf_plan_kr920.cyclo ./Core/nui/ui_rf_plan_kr920.d ./Core/nui/ui_rf_plan_kr920.o ./Core/nui/ui_rf_plan_kr920.su ./Core/nui/ui_ringbuf.cyclo ./Core/nui/ui_ringbuf.d ./Core/nui/ui_ringbuf.o ./Core/nui/ui_ringbuf.su ./Core/nui/ui_time.cyclo ./Core/nui/ui_time.d ./Core/nui/ui_time.o ./Core/nui/ui_time.su ./Core/nui/ui_uart.cyclo ./Core/nui/ui_uart.d ./Core/nui/ui_uart.o ./Core/nui/ui_uart.su

.PHONY: clean-Core-2f-nui

