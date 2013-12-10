
#include <AP_HAL.h>
//#if CONFIG_HAL_BOARD == HAL_BOARD_ROVERBASEBOARD

#include "HAL_RoverBaseboard_Class.h"
#include "AP_HAL_RoverBaseboard_Private.h"

using namespace RoverBaseboard;

static RoverBaseboardUARTDriver uartADriver;
static RoverBaseboardUARTDriver uartBDriver;
static RoverBaseboardUARTDriver uartCDriver;
static RoverBaseboardSemaphore  i2cSemaphore;
static RoverBaseboardI2CDriver  i2cDriver(&i2cSemaphore);
static RoverBaseboardSPIDeviceManager spiDeviceManager;
static RoverBaseboardAnalogIn analogIn;
static RoverBaseboardStorage storageDriver;
static RoverBaseboardGPIO gpioDriver;
static RoverBaseboardRCInput rcinDriver;
static RoverBaseboardRCOutput rcoutDriver;
static RoverBaseboardScheduler schedulerInstance;
static RoverBaseboardUtil utilInstance;

HAL_RoverBaseboard::HAL_RoverBaseboard() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance),
    _member(new RoverBaseboardPrivateMember(123))
{}

void HAL_RoverBaseboard::init(int argc,char* const argv[]) const {
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init(NULL);
    uartA->begin(115200);
    _member->init();
}

const HAL_RoverBaseboard AP_HAL_RoverBaseboard;

//#endif
