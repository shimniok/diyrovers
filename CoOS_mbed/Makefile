# This file was automagically generated by mbed.org. For more information, 
# see http://mbed.org/handbook/Exporting-to-CodeSourcery

GCC_BIN = 
PROJECT = CoOS_mbed
OBJECTS = ./main.o ../CoOS/portable/arch.o ../CoOS/portable/GCC/port.o ../CoOS/kernel/core.o ../CoOS/kernel/event.o ../CoOS/kernel/flag.o ../CoOS/kernel/hook.o ../CoOS/kernel/kernelHeap.o ../CoOS/kernel/mbox.o ../CoOS/kernel/mm.o ../CoOS/kernel/mutex.o ../CoOS/kernel/queue.o ../CoOS/kernel/sem.o ../CoOS/kernel/serviceReq.o ../CoOS/kernel/task.o ../CoOS/kernel/time.o ../CoOS/kernel/timer.o ../CoOS/kernel/utility.o
SYS_OBJECTS = ../mbed/TARGET_LPC1768/TOOLCHAIN_GCC_CS/sys.o ../mbed/TARGET_LPC1768/TOOLCHAIN_GCC_CS/cmsis_nvic.o ../mbed/TARGET_LPC1768/TOOLCHAIN_GCC_CS/system_LPC17xx.o ../mbed/TARGET_LPC1768/TOOLCHAIN_GCC_CS/startup_LPC17xx.o ../mbed/TARGET_LPC1768/TOOLCHAIN_GCC_CS/retarget.o 
INCLUDE_PATHS = -I. -I../CoOS/kernel -I../CoOS/Other -I../CoOS/portable -I../mbed -I../mbed/TARGET_LPC1768 -I../mbed/TARGET_LPC1768/TOOLCHAIN_GCC_CS 
LIBRARY_PATHS = -L../mbed/TARGET_LPC1768/TOOLCHAIN_GCC_CS 
LIBRARIES = -lmbed 
LINKER_SCRIPT = ../mbed/TARGET_LPC1768/TOOLCHAIN_GCC_CS/LPC1768.ld

############################################################################### 
CC = $(GCC_BIN)arm-none-eabi-gcc
CPP = $(GCC_BIN)arm-none-eabi-g++
CC_FLAGS = -c -Os -fno-common -fmessage-length=0 -Wall -fno-exceptions -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections 
ONLY_C_FLAGS = -std=gnu99
ONLY_CPP_FLAGS = -std=gnu++98
CC_SYMBOLS = -DTARGET_LPC1768 -DTARGET_M3 -DTARGET_NXP -DTARGET_LPC176X -DTOOLCHAIN_GCC_CS -DTOOLCHAIN_GCC -D__CORTEX_M3 -DARM_MATH_CM3 


AS = $(GCC_BIN)arm-none-eabi-as

LD = $(GCC_BIN)arm-none-eabi-gcc
LD_FLAGS = -mcpu=cortex-m3 -mthumb -Wl,--gc-sections
LD_SYS_LIBS = -lstdc++ -lsupc++ -lm -lc -lgcc

OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy

all: $(PROJECT).bin

clean:
	rm -f $(PROJECT).bin $(PROJECT).elf $(OBJECTS)

.s.o:
	$(AS)  $(CC_FLAGS) $(CC_SYMBOLS) -o $@ $<

.c.o:
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) $(ONLY_C_FLAGS)   $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) $(ONLY_CPP_FLAGS) $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS)
	$(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ $(LIBRARIES) $(LD_SYS_LIBS) $(LIBRARIES) $(LD_SYS_LIBS)

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

