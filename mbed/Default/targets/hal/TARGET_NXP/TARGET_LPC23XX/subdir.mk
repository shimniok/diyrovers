################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/analogin_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/analogout_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/can_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/ethernet_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/gpio_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/gpio_irq_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/i2c_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/pinmap.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/port_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/pwmout_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/rtc_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/serial_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/spi_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC23XX/us_ticker.c 

C_DEPS += \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/analogin_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/analogout_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/can_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/ethernet_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/gpio_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/gpio_irq_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/i2c_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/pinmap.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/port_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/pwmout_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/rtc_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/serial_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/spi_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC23XX/us_ticker.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_NXP/TARGET_LPC23XX/%.o: ../targets/hal/TARGET_NXP/TARGET_LPC23XX/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


