################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/adc.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/aspep.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/comp.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/cordic.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/dac.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/dma.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/fdcan.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/gpio.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/iwdg.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/main.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_api.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_app_hooks.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_config.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_config_common.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_configuration_registers.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_interface.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_math.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_parameters.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_perf.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_tasks.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_tasks_foc.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mcp.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mcp_config.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/motorcontrol.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/opamp.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/pwm_common.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/pwm_curr_fdbk.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/register_interface.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/regular_conversion_manager.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/stm32_mc_common_it.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/stm32g4xx_hal_msp.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/stm32g4xx_it.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/stm32g4xx_mc_it.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/tim.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/usart.c \
D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/usart_aspep_driver.c 

OBJS += \
./Application/User/adc.o \
./Application/User/aspep.o \
./Application/User/comp.o \
./Application/User/cordic.o \
./Application/User/dac.o \
./Application/User/dma.o \
./Application/User/fdcan.o \
./Application/User/gpio.o \
./Application/User/iwdg.o \
./Application/User/main.o \
./Application/User/mc_api.o \
./Application/User/mc_app_hooks.o \
./Application/User/mc_config.o \
./Application/User/mc_config_common.o \
./Application/User/mc_configuration_registers.o \
./Application/User/mc_interface.o \
./Application/User/mc_math.o \
./Application/User/mc_parameters.o \
./Application/User/mc_perf.o \
./Application/User/mc_tasks.o \
./Application/User/mc_tasks_foc.o \
./Application/User/mcp.o \
./Application/User/mcp_config.o \
./Application/User/motorcontrol.o \
./Application/User/opamp.o \
./Application/User/pwm_common.o \
./Application/User/pwm_curr_fdbk.o \
./Application/User/register_interface.o \
./Application/User/regular_conversion_manager.o \
./Application/User/stm32_mc_common_it.o \
./Application/User/stm32g4xx_hal_msp.o \
./Application/User/stm32g4xx_it.o \
./Application/User/stm32g4xx_mc_it.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o \
./Application/User/tim.o \
./Application/User/usart.o \
./Application/User/usart_aspep_driver.o 

C_DEPS += \
./Application/User/adc.d \
./Application/User/aspep.d \
./Application/User/comp.d \
./Application/User/cordic.d \
./Application/User/dac.d \
./Application/User/dma.d \
./Application/User/fdcan.d \
./Application/User/gpio.d \
./Application/User/iwdg.d \
./Application/User/main.d \
./Application/User/mc_api.d \
./Application/User/mc_app_hooks.d \
./Application/User/mc_config.d \
./Application/User/mc_config_common.d \
./Application/User/mc_configuration_registers.d \
./Application/User/mc_interface.d \
./Application/User/mc_math.d \
./Application/User/mc_parameters.d \
./Application/User/mc_perf.d \
./Application/User/mc_tasks.d \
./Application/User/mc_tasks_foc.d \
./Application/User/mcp.d \
./Application/User/mcp_config.d \
./Application/User/motorcontrol.d \
./Application/User/opamp.d \
./Application/User/pwm_common.d \
./Application/User/pwm_curr_fdbk.d \
./Application/User/register_interface.d \
./Application/User/regular_conversion_manager.d \
./Application/User/stm32_mc_common_it.d \
./Application/User/stm32g4xx_hal_msp.d \
./Application/User/stm32g4xx_it.d \
./Application/User/stm32g4xx_mc_it.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d \
./Application/User/tim.d \
./Application/User/usart.d \
./Application/User/usart_aspep_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/adc.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/adc.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/aspep.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/aspep.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/comp.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/comp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/cordic.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/cordic.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/dac.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/dac.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/dma.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/dma.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/fdcan.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/fdcan.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/gpio.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/gpio.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/iwdg.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/iwdg.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/main.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_api.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_api.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_app_hooks.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_app_hooks.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_config.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_config.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_config_common.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_config_common.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_configuration_registers.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_configuration_registers.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_interface.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_interface.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_math.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_math.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_parameters.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_parameters.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_perf.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_perf.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_tasks.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_tasks.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mc_tasks_foc.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mc_tasks_foc.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mcp.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mcp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/mcp_config.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/mcp_config.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/motorcontrol.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/motorcontrol.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/opamp.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/opamp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/pwm_common.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/pwm_common.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/pwm_curr_fdbk.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/pwm_curr_fdbk.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/register_interface.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/register_interface.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/regular_conversion_manager.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/regular_conversion_manager.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32_mc_common_it.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/stm32_mc_common_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32g4xx_hal_msp.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/stm32g4xx_hal_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32g4xx_it.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/stm32g4xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32g4xx_mc_it.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/stm32g4xx_mc_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/%.o Application/User/%.su Application/User/%.cyclo: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/tim.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/tim.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/usart.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/usart.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/usart_aspep_driver.o: D:/motor/STM32_BLDC_FOC_contorl/B-G431-B-ESC1-Firmware/pully_angle/Src/usart_aspep_driver.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v6.3.0-Full/MotorControl/MCSDK/MCLib/G4xx/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/adc.cyclo ./Application/User/adc.d ./Application/User/adc.o ./Application/User/adc.su ./Application/User/aspep.cyclo ./Application/User/aspep.d ./Application/User/aspep.o ./Application/User/aspep.su ./Application/User/comp.cyclo ./Application/User/comp.d ./Application/User/comp.o ./Application/User/comp.su ./Application/User/cordic.cyclo ./Application/User/cordic.d ./Application/User/cordic.o ./Application/User/cordic.su ./Application/User/dac.cyclo ./Application/User/dac.d ./Application/User/dac.o ./Application/User/dac.su ./Application/User/dma.cyclo ./Application/User/dma.d ./Application/User/dma.o ./Application/User/dma.su ./Application/User/fdcan.cyclo ./Application/User/fdcan.d ./Application/User/fdcan.o ./Application/User/fdcan.su ./Application/User/gpio.cyclo ./Application/User/gpio.d ./Application/User/gpio.o ./Application/User/gpio.su ./Application/User/iwdg.cyclo ./Application/User/iwdg.d ./Application/User/iwdg.o ./Application/User/iwdg.su ./Application/User/main.cyclo ./Application/User/main.d ./Application/User/main.o ./Application/User/main.su ./Application/User/mc_api.cyclo ./Application/User/mc_api.d ./Application/User/mc_api.o ./Application/User/mc_api.su ./Application/User/mc_app_hooks.cyclo ./Application/User/mc_app_hooks.d ./Application/User/mc_app_hooks.o ./Application/User/mc_app_hooks.su ./Application/User/mc_config.cyclo ./Application/User/mc_config.d ./Application/User/mc_config.o ./Application/User/mc_config.su ./Application/User/mc_config_common.cyclo ./Application/User/mc_config_common.d ./Application/User/mc_config_common.o ./Application/User/mc_config_common.su ./Application/User/mc_configuration_registers.cyclo ./Application/User/mc_configuration_registers.d ./Application/User/mc_configuration_registers.o ./Application/User/mc_configuration_registers.su ./Application/User/mc_interface.cyclo ./Application/User/mc_interface.d ./Application/User/mc_interface.o ./Application/User/mc_interface.su ./Application/User/mc_math.cyclo ./Application/User/mc_math.d ./Application/User/mc_math.o ./Application/User/mc_math.su ./Application/User/mc_parameters.cyclo ./Application/User/mc_parameters.d ./Application/User/mc_parameters.o ./Application/User/mc_parameters.su ./Application/User/mc_perf.cyclo ./Application/User/mc_perf.d ./Application/User/mc_perf.o ./Application/User/mc_perf.su ./Application/User/mc_tasks.cyclo ./Application/User/mc_tasks.d ./Application/User/mc_tasks.o ./Application/User/mc_tasks.su ./Application/User/mc_tasks_foc.cyclo ./Application/User/mc_tasks_foc.d ./Application/User/mc_tasks_foc.o ./Application/User/mc_tasks_foc.su ./Application/User/mcp.cyclo ./Application/User/mcp.d ./Application/User/mcp.o ./Application/User/mcp.su ./Application/User/mcp_config.cyclo ./Application/User/mcp_config.d ./Application/User/mcp_config.o ./Application/User/mcp_config.su ./Application/User/motorcontrol.cyclo ./Application/User/motorcontrol.d ./Application/User/motorcontrol.o ./Application/User/motorcontrol.su ./Application/User/opamp.cyclo ./Application/User/opamp.d ./Application/User/opamp.o ./Application/User/opamp.su ./Application/User/pwm_common.cyclo ./Application/User/pwm_common.d ./Application/User/pwm_common.o ./Application/User/pwm_common.su ./Application/User/pwm_curr_fdbk.cyclo ./Application/User/pwm_curr_fdbk.d ./Application/User/pwm_curr_fdbk.o ./Application/User/pwm_curr_fdbk.su ./Application/User/register_interface.cyclo ./Application/User/register_interface.d ./Application/User/register_interface.o ./Application/User/register_interface.su ./Application/User/regular_conversion_manager.cyclo ./Application/User/regular_conversion_manager.d ./Application/User/regular_conversion_manager.o ./Application/User/regular_conversion_manager.su ./Application/User/stm32_mc_common_it.cyclo ./Application/User/stm32_mc_common_it.d ./Application/User/stm32_mc_common_it.o ./Application/User/stm32_mc_common_it.su ./Application/User/stm32g4xx_hal_msp.cyclo ./Application/User/stm32g4xx_hal_msp.d ./Application/User/stm32g4xx_hal_msp.o ./Application/User/stm32g4xx_hal_msp.su ./Application/User/stm32g4xx_it.cyclo ./Application/User/stm32g4xx_it.d ./Application/User/stm32g4xx_it.o ./Application/User/stm32g4xx_it.su ./Application/User/stm32g4xx_mc_it.cyclo ./Application/User/stm32g4xx_mc_it.d ./Application/User/stm32g4xx_mc_it.o ./Application/User/stm32g4xx_mc_it.su ./Application/User/syscalls.cyclo ./Application/User/syscalls.d ./Application/User/syscalls.o ./Application/User/syscalls.su ./Application/User/sysmem.cyclo ./Application/User/sysmem.d ./Application/User/sysmem.o ./Application/User/sysmem.su ./Application/User/tim.cyclo ./Application/User/tim.d ./Application/User/tim.o ./Application/User/tim.su ./Application/User/usart.cyclo ./Application/User/usart.d ./Application/User/usart.o ./Application/User/usart.su ./Application/User/usart_aspep_driver.cyclo ./Application/User/usart_aspep_driver.d ./Application/User/usart_aspep_driver.o ./Application/User/usart_aspep_driver.su

.PHONY: clean-Application-2f-User

