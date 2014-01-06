################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/analogin_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/analogout_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/gpio_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/gpio_irq_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/pinmap.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/port_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/rtc_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/serial_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/sleep.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/spi_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC43XX/us_ticker.c 

C_DEPS += \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/analogin_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/analogout_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/gpio_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/gpio_irq_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/pinmap.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/port_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/rtc_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/serial_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/sleep.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/spi_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC43XX/us_ticker.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_NXP/TARGET_LPC43XX/%.o: ../targets/hal/TARGET_NXP/TARGET_LPC43XX/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


