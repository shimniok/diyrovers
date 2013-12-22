################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS_Library/list.c \
../FreeRTOS_Library/queue.c \
../FreeRTOS_Library/tasks.c \
../FreeRTOS_Library/timers.c 

OBJS += \
./FreeRTOS_Library/list.o \
./FreeRTOS_Library/queue.o \
./FreeRTOS_Library/tasks.o \
./FreeRTOS_Library/timers.o 

C_DEPS += \
./FreeRTOS_Library/list.d \
./FreeRTOS_Library/queue.d \
./FreeRTOS_Library/tasks.d \
./FreeRTOS_Library/timers.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS_Library/%.o: ../FreeRTOS_Library/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -I"/home/mes/Projects/diyrovers/SimpleDemo/FreeRTOS_Library/include" -I"/home/mes/Projects/diyrovers/SimpleDemo/FreeRTOS_Library/portable" -I"/home/mes/Projects/diyrovers/SimpleDemo/CMSISv1p30_LPC17xx/inc" -I"/home/mes/Projects/diyrovers/SimpleDemo" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


