################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/TARGET_LPC11CXX/can_api.c 

C_DEPS += \
./targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/TARGET_LPC11CXX/can_api.d 


# Each subdirectory must supply rules for building sources it contributes
targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/TARGET_LPC11CXX/%.o: ../targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/TARGET_LPC11CXX/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


