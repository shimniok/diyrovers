################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/gpio_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/gpio_irq_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/i2c_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/pinmap.c \
../targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/serial_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/spi_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/us_ticker.c 

C_DEPS += \
./targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/gpio_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/gpio_irq_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/i2c_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/pinmap.d \
./targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/serial_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/spi_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/us_ticker.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/%.o: ../targets/hal/TARGET_NXP/TARGET_LPC81X/TARGET_LPC81X_COMMON/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


