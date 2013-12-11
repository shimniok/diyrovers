
#include "SPIDriver.h"

using namespace RoverBaseboard;

RoverBaseboardSPIDeviceDriver::RoverBaseboardSPIDeviceDriver()
{}

void RoverBaseboardSPIDeviceDriver::init()
{}

AP_HAL::Semaphore* RoverBaseboardSPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

void RoverBaseboardSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{}


void RoverBaseboardSPIDeviceDriver::cs_assert()
{}

void RoverBaseboardSPIDeviceDriver::cs_release()
{}

uint8_t RoverBaseboardSPIDeviceDriver::transfer (uint8_t data)
{
    return 0;
}

void RoverBaseboardSPIDeviceDriver::transfer (const uint8_t *data, uint16_t len)
{
}

RoverBaseboardSPIDeviceManager::RoverBaseboardSPIDeviceManager()
{}

void RoverBaseboardSPIDeviceManager::init(void *)
{}

AP_HAL::SPIDeviceDriver* RoverBaseboardSPIDeviceManager::device(enum AP_HAL::SPIDevice)
{
    return &_device;
}

