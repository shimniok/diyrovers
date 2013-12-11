
#include "RCOutput.h"

using namespace RoverBaseboard;

void RoverBaseboardRCOutput::init(void* machtnichts) {}

void RoverBaseboardRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t RoverBaseboardRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void RoverBaseboardRCOutput::enable_ch(uint8_t ch)
{}

void RoverBaseboardRCOutput::enable_mask(uint32_t chmask)
{}

void RoverBaseboardRCOutput::disable_ch(uint8_t ch)
{}

void RoverBaseboardRCOutput::disable_mask(uint32_t chmask)
{}

void RoverBaseboardRCOutput::write(uint8_t ch, uint16_t period_us)
{}

void RoverBaseboardRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{}

uint16_t RoverBaseboardRCOutput::read(uint8_t ch) {
    return 900;
}

void RoverBaseboardRCOutput::read(uint16_t* period_us, uint8_t len)
{}

