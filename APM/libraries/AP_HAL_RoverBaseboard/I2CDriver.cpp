
#include <AP_HAL.h>
#include "I2CDriver.h"

using namespace RoverBaseboard;

void RoverBaseboardI2CDriver::begin() {}
void RoverBaseboardI2CDriver::end() {}
void RoverBaseboardI2CDriver::setTimeout(uint16_t ms) {}
void RoverBaseboardI2CDriver::setHighSpeed(bool active) {}

uint8_t RoverBaseboardI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{return 0;} 
uint8_t RoverBaseboardI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{return 0;}
uint8_t RoverBaseboardI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 0;}

uint8_t RoverBaseboardI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{return 0;}
uint8_t RoverBaseboardI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{return 0;}
uint8_t RoverBaseboardI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                              uint8_t len, uint8_t* data)
{return 0;}

uint8_t RoverBaseboardI2CDriver::lockup_count() {return 0;}
