
#include "UARTDriver.h"

using namespace RoverBaseboard;

RoverBaseboardUARTDriver::RoverBaseboardUARTDriver() {}

void RoverBaseboardUARTDriver::begin(uint32_t b) {}
void RoverBaseboardUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void RoverBaseboardUARTDriver::end() {}
void RoverBaseboardUARTDriver::flush() {}
bool RoverBaseboardUARTDriver::is_initialized() { return false; }
void RoverBaseboardUARTDriver::set_blocking_writes(bool blocking) {}
bool RoverBaseboardUARTDriver::tx_pending() { return false; }

/* RoverBaseboard implementations of Stream virtual methods */
int16_t RoverBaseboardUARTDriver::available() { return 0; }
int16_t RoverBaseboardUARTDriver::txspace() { return 1; }
int16_t RoverBaseboardUARTDriver::read() { return -1; }

/* RoverBaseboard implementations of Print virtual methods */
size_t RoverBaseboardUARTDriver::write(uint8_t c) { return 0; }

size_t RoverBaseboardUARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}
