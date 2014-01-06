################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/cmsis/TARGET_NXP/TARGET_LPC176X/cmsis_nvic.c \
../targets/cmsis/TARGET_NXP/TARGET_LPC176X/system_LPC17xx.c 

C_DEPS += \
./targets/cmsis/TARGET_NXP/TARGET_LPC176X/cmsis_nvic.d \
./targets/cmsis/TARGET_NXP/TARGET_LPC176X/system_LPC17xx.d 


# Each subdirectory must supply rules for building sources it contributes
targets/cmsis/TARGET_NXP/TARGET_LPC176X/%.o: ../targets/cmsis/TARGET_NXP/TARGET_LPC176X/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


