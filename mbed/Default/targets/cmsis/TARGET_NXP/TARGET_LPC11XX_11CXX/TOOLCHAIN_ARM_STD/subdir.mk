################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/TOOLCHAIN_ARM_STD/sys.cpp 

CPP_DEPS += \
./targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/TOOLCHAIN_ARM_STD/sys.d 


# Each subdirectory must supply rules for building sources it contributes
targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/TOOLCHAIN_ARM_STD/%.o: ../targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/TOOLCHAIN_ARM_STD/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C++ Compiler'
	arm-none-eabi-g++ -O2 -Wall -Wa,-adhlns="$@.lst" -fno-exceptions -fno-rtti -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


