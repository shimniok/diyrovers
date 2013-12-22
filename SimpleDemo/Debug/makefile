################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include FreeRTOS_Library/portable/subdir.mk
-include FreeRTOS_Library/demo_code/subdir.mk
-include FreeRTOS_Library/subdir.mk
-include CMSISv1p30_LPC17xx/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SimpleDemo.axf

# Tool invocations
SimpleDemo.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -Xlinker -Map="SimpleDemo.map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T "SimpleDemo_Debug.ld" -o "SimpleDemo.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) SimpleDemo.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "SimpleDemo.axf"; # arm-none-eabi-objcopy -O binary "SimpleDemo.axf" "SimpleDemo.bin" ; checksum -p LPC1769 -d "SimpleDemo.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
