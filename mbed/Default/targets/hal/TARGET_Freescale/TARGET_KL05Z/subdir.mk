################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/analogin_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/analogout_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/gpio_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/gpio_irq_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/i2c_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/pinmap.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/port_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/pwmout_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/rtc_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/serial_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/spi_api.c \
../targets/hal/TARGET_Freescale/TARGET_KL05Z/us_ticker.c 

C_DEPS += \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/analogin_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/analogout_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/gpio_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/gpio_irq_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/i2c_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/pinmap.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/port_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/pwmout_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/rtc_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/serial_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/spi_api.d \
./targets/hal/TARGET_Freescale/TARGET_KL05Z/us_ticker.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_Freescale/TARGET_KL05Z/%.o: ../targets/hal/TARGET_Freescale/TARGET_KL05Z/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


