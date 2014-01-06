################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/cmsis/TARGET_NXP/TARGET_LPC23XX/cmsis_nvic.c \
../targets/cmsis/TARGET_NXP/TARGET_LPC23XX/core_arm7.c \
../targets/cmsis/TARGET_NXP/TARGET_LPC23XX/system_LPC23xx.c \
../targets/cmsis/TARGET_NXP/TARGET_LPC23XX/vector_realmonitor.c 

C_DEPS += \
./targets/cmsis/TARGET_NXP/TARGET_LPC23XX/cmsis_nvic.d \
./targets/cmsis/TARGET_NXP/TARGET_LPC23XX/core_arm7.d \
./targets/cmsis/TARGET_NXP/TARGET_LPC23XX/system_LPC23xx.d \
./targets/cmsis/TARGET_NXP/TARGET_LPC23XX/vector_realmonitor.d 


# Each subdirectory must supply rules for building sources it contributes
targets/cmsis/TARGET_NXP/TARGET_LPC23XX/%.o: ../targets/cmsis/TARGET_NXP/TARGET_LPC23XX/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


