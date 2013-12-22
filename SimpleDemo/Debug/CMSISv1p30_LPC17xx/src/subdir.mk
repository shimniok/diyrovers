################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSISv1p30_LPC17xx/src/core_cm3.c \
../CMSISv1p30_LPC17xx/src/system_LPC17xx.c 

OBJS += \
./CMSISv1p30_LPC17xx/src/core_cm3.o \
./CMSISv1p30_LPC17xx/src/system_LPC17xx.o 

C_DEPS += \
./CMSISv1p30_LPC17xx/src/core_cm3.d \
./CMSISv1p30_LPC17xx/src/system_LPC17xx.d 


# Each subdirectory must supply rules for building sources it contributes
CMSISv1p30_LPC17xx/src/%.o: ../CMSISv1p30_LPC17xx/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -I"/home/mes/Projects/diyrovers/SimpleDemo/FreeRTOS_Library/include" -I"/home/mes/Projects/diyrovers/SimpleDemo/FreeRTOS_Library/portable" -I"/home/mes/Projects/diyrovers/SimpleDemo/CMSISv1p30_LPC17xx/inc" -I"/home/mes/Projects/diyrovers/SimpleDemo" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


