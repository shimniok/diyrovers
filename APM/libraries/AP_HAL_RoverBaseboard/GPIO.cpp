
#include "GPIO.h"

using namespace RoverBaseboard;

RoverBaseboardGPIO::RoverBaseboardGPIO()
{}

void RoverBaseboardGPIO::init()
{}

void RoverBaseboardGPIO::pinMode(uint8_t pin, uint8_t output)
{}

int8_t RoverBaseboardGPIO::analogPinToDigitalPin(uint8_t pin)
{
	return -1;
}


uint8_t RoverBaseboardGPIO::read(uint8_t pin) {
    return 0;
}

void RoverBaseboardGPIO::write(uint8_t pin, uint8_t value)
{}

void RoverBaseboardGPIO::toggle(uint8_t pin)
{}

/* Alternative interface: */
AP_HAL::DigitalSource* RoverBaseboardGPIO::channel(uint16_t n) {
    return new RoverBaseboardDigitalSource(0);
}

/* Interrupt interface: */
bool RoverBaseboardGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool RoverBaseboardGPIO::usb_connected(void)
{
    return false;
}

RoverBaseboardDigitalSource::RoverBaseboardDigitalSource(uint8_t v) :
    _v(v)
{}

void RoverBaseboardDigitalSource::mode(uint8_t output)
{}

uint8_t RoverBaseboardDigitalSource::read() {
    return _v;
}

void RoverBaseboardDigitalSource::write(uint8_t value) {
    _v = value;
}

void RoverBaseboardDigitalSource::toggle() {
    _v = !_v;
}
