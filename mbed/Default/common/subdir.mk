################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../common/BusIn.cpp \
../common/BusInOut.cpp \
../common/BusOut.cpp \
../common/CAN.cpp \
../common/CallChain.cpp \
../common/Ethernet.cpp \
../common/FileBase.cpp \
../common/FileLike.cpp \
../common/FilePath.cpp \
../common/FileSystemLike.cpp \
../common/FunctionPointer.cpp \
../common/I2C.cpp \
../common/I2CSlave.cpp \
../common/InterruptIn.cpp \
../common/InterruptManager.cpp \
../common/LocalFileSystem.cpp \
../common/RawSerial.cpp \
../common/SPI.cpp \
../common/SPISlave.cpp \
../common/Serial.cpp \
../common/SerialBase.cpp \
../common/Stream.cpp \
../common/Ticker.cpp \
../common/Timeout.cpp \
../common/Timer.cpp \
../common/TimerEvent.cpp \
../common/retarget.cpp 

C_SRCS += \
../common/board.c \
../common/exit.c \
../common/mbed_interface.c \
../common/pinmap_common.c \
../common/rtc_time.c \
../common/semihost_api.c \
../common/us_ticker_api.c \
../common/wait_api.c 

C_DEPS += \
./common/board.d \
./common/exit.d \
./common/mbed_interface.d \
./common/pinmap_common.d \
./common/rtc_time.d \
./common/semihost_api.d \
./common/us_ticker_api.d \
./common/wait_api.d 

CPP_DEPS += \
./common/BusIn.d \
./common/BusInOut.d \
./common/BusOut.d \
./common/CAN.d \
./common/CallChain.d \
./common/Ethernet.d \
./common/FileBase.d \
./common/FileLike.d \
./common/FilePath.d \
./common/FileSystemLike.d \
./common/FunctionPointer.d \
./common/I2C.d \
./common/I2CSlave.d \
./common/InterruptIn.d \
./common/InterruptManager.d \
./common/LocalFileSystem.d \
./common/RawSerial.d \
./common/SPI.d \
./common/SPISlave.d \
./common/Serial.d \
./common/SerialBase.d \
./common/Stream.d \
./common/Ticker.d \
./common/Timeout.d \
./common/Timer.d \
./common/TimerEvent.d \
./common/retarget.d 


# Each subdirectory must supply rules for building sources it contributes
common/%.o: ../common/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C++ Compiler'
	arm-none-eabi-g++ -O2 -Wall -Wa,-adhlns="$@.lst" -fno-exceptions -fno-rtti -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

common/%.o: ../common/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C Compiler'
	arm-none-eabi-gcc -O2 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


