
#ifndef __AP_HAL_ROVERBASEBOARD_NAMESPACE_H__
#define __AP_HAL_ROVERBASEBOARD_NAMESPACE_H__

/* While not strictly required, names inside the namespace are prefixed
 * for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace RoverBaseboard {
    class RoverBaseboardUARTDriver;
    class RoverBaseboardI2CDriver;
    class RoverBaseboardSPIDeviceManager;
    class RoverBaseboardSPIDeviceDriver;
    class RoverBaseboardAnalogSource;
    class RoverBaseboardAnalogIn;
    class RoverBaseboardStorage;
    class RoverBaseboardGPIO;
    class RoverBaseboardDigitalSource;
    class RoverBaseboardRCInput;
    class RoverBaseboardRCOutput;
    class RoverBaseboardSemaphore;
    class RoverBaseboardScheduler;
    class RoverBaseboardUtil;
    class RoverBaseboardPrivateMember;
}

#endif // __AP_HAL_ROVERBASEBOARD_NAMESPACE_H__

