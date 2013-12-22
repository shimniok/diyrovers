################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/CMSISv1p30_LPC17xx/src/core_cm3.c \
../FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/CMSISv1p30_LPC17xx/src/system_LPC17xx.c 

OBJS += \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/CMSISv1p30_LPC17xx/src/core_cm3.o \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/CMSISv1p30_LPC17xx/src/system_LPC17xx.o 

C_DEPS += \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/CMSISv1p30_LPC17xx/src/core_cm3.d \
./FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/CMSISv1p30_LPC17xx/src/system_LPC17xx.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/CMSISv1p30_LPC17xx/src/%.o: ../FreeRTOS-simple-demo-for-LPCXpresso-LPC1768/CMSISv1p30_LPC17xx/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


