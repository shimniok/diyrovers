################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/analogin_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/gpio_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/gpio_irq_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/i2c_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/pinmap.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/port_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/pwmout_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/serial_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/sleep.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/spi_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC13XX/us_ticker.c 

C_DEPS += \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/analogin_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/gpio_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/gpio_irq_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/i2c_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/pinmap.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/port_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/pwmout_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/serial_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/sleep.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/spi_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC13XX/us_ticker.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_NXP/TARGET_LPC13XX/%.o: ../targets/hal/TARGET_NXP/TARGET_LPC13XX/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


