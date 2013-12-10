#include "AnalogIn.h"

using namespace RoverBaseboard;

RoverBaseboardAnalogSource::RoverBaseboardAnalogSource(float v) :
    _v(v)
{}

float RoverBaseboardAnalogSource::read_average() {
    return _v;
}

float RoverBaseboardAnalogSource::voltage_average() {
    return 5.0 * _v / 1024.0;
}

float RoverBaseboardAnalogSource::voltage_latest() {
    return 5.0 * _v / 1024.0;
}

float RoverBaseboardAnalogSource::read_latest() {
    return _v;
}

void RoverBaseboardAnalogSource::set_pin(uint8_t p)
{}

void RoverBaseboardAnalogSource::set_stop_pin(uint8_t p)
{}

void RoverBaseboardAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

RoverBaseboardAnalogIn::RoverBaseboardAnalogIn()
{}

void RoverBaseboardAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* RoverBaseboardAnalogIn::channel(int16_t n) {
    return new RoverBaseboardAnalogSource(1.11);
}


