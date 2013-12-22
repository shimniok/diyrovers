################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/list.c \
../FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/queue.c \
../FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/tasks.c \
../FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/timers.c 

OBJS += \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/list.o \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/queue.o \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/tasks.o \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/timers.o 

C_DEPS += \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/list.d \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/queue.d \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/tasks.d \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/timers.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/%.o: ../FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/FreeRTOS_Library/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


