################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS_Library/demo_code/basic_io.c \
../FreeRTOS_Library/demo_code/consoleprint.c 

OBJS += \
./FreeRTOS_Library/demo_code/basic_io.o \
./FreeRTOS_Library/demo_code/consoleprint.o 

C_DEPS += \
./FreeRTOS_Library/demo_code/basic_io.d \
./FreeRTOS_Library/demo_code/consoleprint.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS_Library/demo_code/%.o: ../FreeRTOS_Library/demo_code/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -I"/home/mes/Projects/diyrovers/SimpleDemo/FreeRTOS_Library/include" -I"/home/mes/Projects/diyrovers/SimpleDemo/FreeRTOS_Library/portable" -I"/home/mes/Projects/diyrovers/SimpleDemo/CMSISv1p30_LPC17xx/inc" -I"/home/mes/Projects/diyrovers/SimpleDemo" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


