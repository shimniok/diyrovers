
#include "RCInput.h"

using namespace RoverBaseboard;
RoverBaseboardRCInput::RoverBaseboardRCInput()
{}

void RoverBaseboardRCInput::init(void* machtnichts)
{}

uint8_t RoverBaseboardRCInput::valid_channels() {
    return 0;
}

uint16_t RoverBaseboardRCInput::read(uint8_t ch) {
    if (ch == 2) return 900; /* throttle should be low, for safety */
    else return 1500;
}

uint8_t RoverBaseboardRCInput::read(uint16_t* periods, uint8_t len) {
    for (uint8_t i = 0; i < len; i++){
        if (i == 2) periods[i] = 900;
        else periods[i] = 1500;
    }
    return len;
}

bool RoverBaseboardRCInput::set_overrides(int16_t *overrides, uint8_t len) {
    return true;
}

bool RoverBaseboardRCInput::set_override(uint8_t channel, int16_t override) {
    return true;
}

void RoverBaseboardRCInput::clear_overrides()
{}

