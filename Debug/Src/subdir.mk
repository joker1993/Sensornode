################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/can.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c 

OBJS += \
./Src/can.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o 

C_DEPS += \
./Src/can.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L496xx -I"C:/Users/Kristof/Documents/STM32L496RE_workspace/Sensornode/Inc" -I"C:/Users/Kristof/Documents/STM32L496RE_workspace/Sensornode/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/Kristof/Documents/STM32L496RE_workspace/Sensornode/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Kristof/Documents/STM32L496RE_workspace/Sensornode/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/Kristof/Documents/STM32L496RE_workspace/Sensornode/Drivers/CMSIS/Include" -I"C:/Users/Kristof/Documents/STM32L496RE_workspace/Sensornode/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

