
#ifndef __AP_HAL_EMPTY_SPIDRIVER_H__
#define __AP_HAL_EMPTY_SPIDRIVER_H__

#include <AP_HAL_RoverBaseboard.h>
#include "Semaphores.h"

class RoverBaseboard::RoverBaseboardSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    RoverBaseboardSPIDeviceDriver();
    void init();
    AP_HAL::Semaphore* get_semaphore();
    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
private:
    RoverBaseboardSemaphore _semaphore;
};

class RoverBaseboard::RoverBaseboardSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    RoverBaseboardSPIDeviceManager();
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);
private:
    RoverBaseboardSPIDeviceDriver _device;
};

#endif // __AP_HAL_EMPTY_SPIDRIVER_H__
