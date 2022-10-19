################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/adc_callback.c \
../Core/Lib/i2c_dev.c \
../Core/Lib/motor.c \
../Core/Lib/radio.c 

OBJS += \
./Core/Lib/adc_callback.o \
./Core/Lib/i2c_dev.o \
./Core/Lib/motor.o \
./Core/Lib/radio.o 

C_DEPS += \
./Core/Lib/adc_callback.d \
./Core/Lib/i2c_dev.d \
./Core/Lib/motor.d \
./Core/Lib/radio.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/%.o: ../Core/Lib/%.c Core/Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Lib -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

