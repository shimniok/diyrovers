################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/analogin_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/gpio_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/gpio_irq_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/i2c_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/pinmap.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/port_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/pwmout_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/serial_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/sleep.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/spi_api.c \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/us_ticker.c 

C_DEPS += \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/analogin_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/gpio_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/gpio_irq_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/i2c_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/pinmap.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/port_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/pwmout_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/serial_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/sleep.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/spi_api.d \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/us_ticker.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/%.o: ../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


