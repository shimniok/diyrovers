
#ifndef __AP_HAL_ROVERBASEBOARD_CLASS_H__
#define __AP_HAL_ROVERBASEBOARD_CLASS_H__

#include <AP_HAL.h>

#include "AP_HAL_RoverBaseboard_Namespace.h"
#include "PrivateMember.h"

class HAL_RoverBaseboard : public AP_HAL::HAL {
public:
    HAL_RoverBaseboard();
    void init(int argc, char * const * argv) const;
private:
    RoverBaseboard::RoverBaseboardPrivateMember *_member;
};

extern const HAL_RoverBaseboard AP_HAL_RoverBaseboard;

#endif // __AP_HAL_ROVERBASEBOARD_CLASS_H__

