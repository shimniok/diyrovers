################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/analogin_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/analogout_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/gpio_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/gpio_irq_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/i2c_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/pinmap.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/port_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/pwmout_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/rtc_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/serial_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/spi_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL46Z/us_ticker.c 

C_DEPS += \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/analogin_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/analogout_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/gpio_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/gpio_irq_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/i2c_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/pinmap.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/port_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/pwmout_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/rtc_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/serial_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/spi_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL46Z/us_ticker.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_Freescale/TARGET_KL46Z/%.o: ../targets/hal/TARGET_Freescale/TARGET_KL46Z/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


