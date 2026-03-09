################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/ui/nd_app.c \
../Core/ui/nd_sensors.c \
../Core/ui/ui_ble.c \
../Core/ui/ui_cmd.c \
../Core/ui/ui_config.c \
../Core/ui/ui_core.c \
../Core/ui/ui_crc16.c \
../Core/ui/ui_gpio.c \
../Core/ui/ui_hal_callbacks.c \
../Core/ui/ui_lpm.c \
../Core/ui/ui_packets.c \
../Core/ui/ui_rf_plan_kr920.c \
../Core/ui/ui_ringbuf.c \
../Core/ui/ui_time.c \
../Core/ui/ui_uart.c 

OBJS += \
./Core/ui/nd_app.o \
./Core/ui/nd_sensors.o \
./Core/ui/ui_ble.o \
./Core/ui/ui_cmd.o \
./Core/ui/ui_config.o \
./Core/ui/ui_core.o \
./Core/ui/ui_crc16.o \
./Core/ui/ui_gpio.o \
./Core/ui/ui_hal_callbacks.o \
./Core/ui/ui_lpm.o \
./Core/ui/ui_packets.o \
./Core/ui/ui_rf_plan_kr920.o \
./Core/ui/ui_ringbuf.o \
./Core/ui/ui_time.o \
./Core/ui/ui_uart.o 

C_DEPS += \
./Core/ui/nd_app.d \
./Core/ui/nd_sensors.d \
./Core/ui/ui_ble.d \
./Core/ui/ui_cmd.d \
./Core/ui/ui_config.d \
./Core/ui/ui_core.d \
./Core/ui/ui_crc16.d \
./Core/ui/ui_gpio.d \
./Core/ui/ui_hal_callbacks.d \
./Core/ui/ui_lpm.d \
./Core/ui/ui_packets.d \
./Core/ui/ui_rf_plan_kr920.d \
./Core/ui/ui_ringbuf.d \
./Core/ui/ui_time.d \
./Core/ui/ui_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/ui/%.o Core/ui/%.su Core/ui/%.cyclo: ../Core/ui/%.c Core/ui/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../Core/Inc -I../SubGHz_Phy/App -I../SubGHz_Phy/Target -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Drivers/CMSIS/Include -I"D:/work26/nd/Core/ui" -Oz -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-ui

clean-Core-2f-ui:
	-$(RM) ./Core/ui/nd_app.cyclo ./Core/ui/nd_app.d ./Core/ui/nd_app.o ./Core/ui/nd_app.su ./Core/ui/nd_sensors.cyclo ./Core/ui/nd_sensors.d ./Core/ui/nd_sensors.o ./Core/ui/nd_sensors.su ./Core/ui/ui_ble.cyclo ./Core/ui/ui_ble.d ./Core/ui/ui_ble.o ./Core/ui/ui_ble.su ./Core/ui/ui_cmd.cyclo ./Core/ui/ui_cmd.d ./Core/ui/ui_cmd.o ./Core/ui/ui_cmd.su ./Core/ui/ui_config.cyclo ./Core/ui/ui_config.d ./Core/ui/ui_config.o ./Core/ui/ui_config.su ./Core/ui/ui_core.cyclo ./Core/ui/ui_core.d ./Core/ui/ui_core.o ./Core/ui/ui_core.su ./Core/ui/ui_crc16.cyclo ./Core/ui/ui_crc16.d ./Core/ui/ui_crc16.o ./Core/ui/ui_crc16.su ./Core/ui/ui_gpio.cyclo ./Core/ui/ui_gpio.d ./Core/ui/ui_gpio.o ./Core/ui/ui_gpio.su ./Core/ui/ui_hal_callbacks.cyclo ./Core/ui/ui_hal_callbacks.d ./Core/ui/ui_hal_callbacks.o ./Core/ui/ui_hal_callbacks.su ./Core/ui/ui_lpm.cyclo ./Core/ui/ui_lpm.d ./Core/ui/ui_lpm.o ./Core/ui/ui_lpm.su ./Core/ui/ui_packets.cyclo ./Core/ui/ui_packets.d ./Core/ui/ui_packets.o ./Core/ui/ui_packets.su ./Core/ui/ui_rf_plan_kr920.cyclo ./Core/ui/ui_rf_plan_kr920.d ./Core/ui/ui_rf_plan_kr920.o ./Core/ui/ui_rf_plan_kr920.su ./Core/ui/ui_ringbuf.cyclo ./Core/ui/ui_ringbuf.d ./Core/ui/ui_ringbuf.o ./Core/ui/ui_ringbuf.su ./Core/ui/ui_time.cyclo ./Core/ui/ui_time.d ./Core/ui/ui_time.o ./Core/ui/ui_time.su ./Core/ui/ui_uart.cyclo ./Core/ui/ui_uart.d ./Core/ui/ui_uart.o ./Core/ui/ui_uart.su

.PHONY: clean-Core-2f-ui

