################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../cr_startup_lpc176x.c \
../main.c 

OBJS += \
./cr_startup_lpc176x.o \
./main.o 

C_DEPS += \
./cr_startup_lpc176x.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -D__USE_CMSIS=CMSISv2_LPC17xx -DNDEBUG -D__CODE_RED -I"/home/mes/Projects/diyrovers/CMSISv2_LPC17xx/inc" -I"/home/mes/Projects/diyrovers/CMSIS_CORE_LPC17xx/inc" -O2 -g -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


