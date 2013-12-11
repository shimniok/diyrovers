/**
 * Copyright (c) 2011 Pololu Corporation.  For more information, see
 * 
 * http://www.pololu.com/
 * http://forum.pololu.com/
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
 
#include "mbed.h"
#include <L3G4200D.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address, 
// and sets the last bit correctly based on reads and writes
// mbed I2C libraries take the 7-bit address shifted left 1 bit
// #define GYR_ADDRESS (0xD2 >> 1)
#define GYR_ADDRESS 0xD2

// Public Methods //////////////////////////////////////////////////////////////

// Constructor
L3G4200D::L3G4200D(PinName sda, PinName scl):
    _device(sda, scl)
{
    _device.frequency(400000);
    // Turns on the L3G4200D's gyro and places it in normal mode.
    // 0x0F = 0b00001111
    // Normal power mode, all axes enabled
    writeReg(L3G4200D_CTRL_REG1, 0x0F);
    writeReg(L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale

}

// Writes a gyro register
void L3G4200D::writeReg(byte reg, byte value)
{
    data[0] = reg;
    data[1] = value;
    
    _device.write(GYR_ADDRESS, data, 2);
}

// Reads a gyro register
byte L3G4200D::readReg(byte reg)
{
    byte value = 0;
    
    _device.write(GYR_ADDRESS, &reg, 1);
    _device.read(GYR_ADDRESS, &value, 1);

    return value;
}

// Reads the 3 gyro channels and stores them in vector g
void L3G4200D::read(int g[3])
{
    // assert the MSB of the address to get the gyro 
    // to do slave-transmit subaddress updating.
    data[0] = L3G4200D_OUT_X_L | (1 << 7);
    _device.write(GYR_ADDRESS, data, 1); 

//    Wire.requestFrom(GYR_ADDRESS, 6);
//    while (Wire.available() < 6);
    
    _device.read(GYR_ADDRESS, data, 6); 

    uint8_t xla = data[0];
    uint8_t xha = data[1];
    uint8_t yla = data[2];
    uint8_t yha = data[3];
    uint8_t zla = data[4];
    uint8_t zha = data[5];

    g[0] = (short) (xha << 8 | xla);
    g[1] = (short) (yha << 8 | yla);
    g[2] = (short) (zha << 8 | zla);
}

byte L3G4200D::readTemp(void) {
    return readReg(L3G4200D_OUT_TEMP);
}
