
#include "GPIO.h"

/**
 *  On RoverBaseboard Rev 0.5 the following is the GPIO pin mapping.
 *  Don't ask why I did it this way. Cuz I don't know. :)
 *
 *  D0 - P0.18
 *  D1 - P0.17
 *  D2 - P0.21
 *  D7 - P2.12
 *  D8 - P2.11
 *
 *  It might be just as easy to use the CMSIS stuff here until we edit mbed PinNames.h for the LPCXpresso.
 *
 * Example:
 *  // Set P0_22 to 00 - GPIO
 *  LPC_PINCON->PINSEL1 &= (~(3 << 12));
 *  // Set GPIO - P0_22 - to be output
 *  LPC_GPIO0->FIODIR |= (1 << 22);
 *
 *  volatile static uint32_t i;
 *  while (1) {
 *      LPC_GPIO0->FIOSET = (1 << 22); // Turn LED2 on
 *      for (i = 0; i < 1000000; i++);
 *      LPC_GPIO0->FIOCLR = (1 << 22); // Turn LED2 off
 *      for (i = 0; i < 1000000; i++);
 *  }
 */

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
