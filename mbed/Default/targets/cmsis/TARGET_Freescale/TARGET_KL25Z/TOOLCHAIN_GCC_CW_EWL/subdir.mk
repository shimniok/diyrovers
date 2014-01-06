################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../targets/cmsis/TARGET_Freescale/TARGET_KL25Z/TOOLCHAIN_GCC_CW_EWL/startup_MKL25Z4.c 

C_DEPS += \
./targets/cmsis/TARGET_Freescale/TARGET_KL25Z/TOOLCHAIN_GCC_CW_EWL/startup_MKL25Z4.d 


# Each subdirectory must supply rules for building sources it contributes
targets/cmsis/TARGET_Freescale/TARGET_KL25Z/TOOLCHAIN_GCC_CW_EWL/%.o: ../targets/cmsis/TARGET_Freescale/TARGET_KL25Z/TOOLCHAIN_GCC_CW_EWL/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


