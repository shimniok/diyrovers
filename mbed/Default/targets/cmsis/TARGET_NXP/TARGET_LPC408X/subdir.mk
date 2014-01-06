################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/cmsis/TARGET_NXP/TARGET_LPC408X/cmsis_nvic.c \
../targets/cmsis/TARGET_NXP/TARGET_LPC408X/system_LPC407x_8x_177x_8x.c 

C_DEPS += \
./targets/cmsis/TARGET_NXP/TARGET_LPC408X/cmsis_nvic.d \
./targets/cmsis/TARGET_NXP/TARGET_LPC408X/system_LPC407x_8x_177x_8x.d 


# Each subdirectory must supply rules for building sources it contributes
targets/cmsis/TARGET_NXP/TARGET_LPC408X/%.o: ../targets/cmsis/TARGET_NXP/TARGET_LPC408X/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


