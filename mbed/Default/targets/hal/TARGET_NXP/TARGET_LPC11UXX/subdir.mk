################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/analogin_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/gpio_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/gpio_irq_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/i2c_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/pinmap.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/port_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/pwmout_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/serial_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/sleep.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/spi_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11UXX/us_ticker.c 

C_DEPS += \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/analogin_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/gpio_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/gpio_irq_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/i2c_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/pinmap.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/port_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/pwmout_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/serial_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/sleep.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/spi_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11UXX/us_ticker.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_NXP/TARGET_LPC11UXX/%.o: ../targets/hal/TARGET_NXP/TARGET_LPC11UXX/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


