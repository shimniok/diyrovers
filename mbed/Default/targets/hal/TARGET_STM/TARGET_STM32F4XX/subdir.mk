################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_STM/TARGET_STM32F4XX/analogin_api.c \
../targets/hal/TARGET_STM/TARGET_STM32F4XX/gpio_api.c \
../targets/hal/TARGET_STM/TARGET_STM32F4XX/i2c_api.c \
../targets/hal/TARGET_STM/TARGET_STM32F4XX/pinmap.c \
../targets/hal/TARGET_STM/TARGET_STM32F4XX/port_api.c \
../targets/hal/TARGET_STM/TARGET_STM32F4XX/spi_api.c \
../targets/hal/TARGET_STM/TARGET_STM32F4XX/us_ticker.c 

C_DEPS += \
./targets/hal/TARGET_STM/TARGET_STM32F4XX/analogin_api.d \
./targets/hal/TARGET_STM/TARGET_STM32F4XX/gpio_api.d \
./targets/hal/TARGET_STM/TARGET_STM32F4XX/i2c_api.d \
./targets/hal/TARGET_STM/TARGET_STM32F4XX/pinmap.d \
./targets/hal/TARGET_STM/TARGET_STM32F4XX/port_api.d \
./targets/hal/TARGET_STM/TARGET_STM32F4XX/spi_api.d \
./targets/hal/TARGET_STM/TARGET_STM32F4XX/us_ticker.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_STM/TARGET_STM32F4XX/%.o: ../targets/hal/TARGET_STM/TARGET_STM32F4XX/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


