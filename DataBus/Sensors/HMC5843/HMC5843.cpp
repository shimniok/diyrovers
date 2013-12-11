/**
 * @author Jose R. Padron
 * @author Used HMC6352 library  developed by Aaron Berk as template
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * Honeywell HMC5843digital compass.
 *
 * Datasheet:
 *
 * http://www.ssec.honeywell.com/magnetic/datasheets/HMC5843.pdf
 */

/**
 * Includes
 */
#include "HMC5843.h"

HMC5843::HMC5843(PinName sda, PinName scl) 
{
    i2c_ = new I2C(sda, scl);
    //100KHz, as specified by the datasheet.
    i2c_->frequency(100000);
    setOpMode(HMC5843_CONTINUOUS, HMC5843_10HZ_NORMAL, HMC5843_1_0GA);

    return;
}


void HMC5843::setOffset(float x, float y, float z)
{
    return;
}


void HMC5843::setScale(float x, float y, float z)
{
    return;
}
     

void HMC5843::setSleepMode()
{
    write(HMC5843_MODE, HMC5843_SLEEP);

    return;
}


void HMC5843::setDefault(void)
{
   write(HMC5843_CONFIG_A,HMC5843_10HZ_NORMAL);
   write(HMC5843_CONFIG_B,HMC5843_1_0GA);
   write(HMC5843_MODE,HMC5843_CONTINUOUS);
   wait_ms(100);
   
   return;
}


void HMC5843::setOpMode(int mode, int ConfigA, int ConfigB)
{      
    write(HMC5843_CONFIG_A,ConfigA);
    write(HMC5843_CONFIG_B,ConfigB);
    write(HMC5843_MODE,mode);
    
    return;
}


void HMC5843::write(int address, int data)
{
    char tx[2];
   
    tx[0]=address;
    tx[1]=data;

    i2c_->write(HMC5843_I2C_WRITE,tx,2);
   
    wait_ms(100);

    return;
}


void HMC5843::read(int m[3])
{
    readData(m);

    return;
}

void HMC5843::readMag(int m[3])
{
    readData(m);

    return;
}

void HMC5843::readData(int readings[3])
{
    char tx[1];
    char rx[2];
    
    
    tx[0]=HMC5843_X_MSB;
    i2c_->write(HMC5843_I2C_READ,tx,1);
    i2c_->read(HMC5843_I2C_READ,rx,2);
    readings[0]= (short)(rx[0]<<8|rx[1]);

     
    tx[0]=HMC5843_Y_MSB;
    i2c_->write(HMC5843_I2C_READ,tx,1);
    i2c_->read(HMC5843_I2C_READ,rx,2);
    readings[1]= (short) (rx[0]<<8|rx[1]);
     
    tx[0]=HMC5843_Z_MSB;
    i2c_->write(HMC5843_I2C_READ,tx,1);
    i2c_->read(HMC5843_I2C_READ,rx,2);
    readings[2]= (short) (rx[0]<<8|rx[1]);
    
    return;
}

int HMC5843::getMx()
{
    char tx[1];
    char rx[2];
    
    
    tx[0]=HMC5843_X_MSB;
    i2c_->write(HMC5843_I2C_READ,tx,1);
    i2c_->read(HMC5843_I2C_READ,rx,2);
    
    return (short) rx[0]<<8|rx[1];
}

int HMC5843::getMy() 
{
    char tx[1];
    char rx[2];
    
    
    tx[0]=HMC5843_Y_MSB;
    i2c_->write(HMC5843_I2C_READ,tx,1);
    i2c_->read(HMC5843_I2C_READ,rx,2);
    
    return (short) rx[0]<<8|rx[1];
}


int HMC5843::getMz(){

    char tx[1];
    char rx[2];
    
    
    tx[0]=HMC5843_Z_MSB;
    i2c_->write(HMC5843_I2C_READ,tx,1);
    i2c_->read(HMC5843_I2C_READ,rx,2);
    
    return (short) rx[0]<<8|rx[1];
}


int HMC5843::getStatus(void)
{
    return 0;
}


void HMC5843::getAddress(char *buffer)
{    
   char rx[3];
   char tx[1];
   tx[0]=HMC5843_IDENT_A;
    
       
    i2c_->write(HMC5843_I2C_WRITE, tx,1);
   
    wait_ms(1);
    
    i2c_->read(HMC5843_I2C_READ,rx,3);
    
    buffer[0]=rx[0];
    buffer[1]=rx[1];
    buffer[2]=rx[2];
    
    return;
}

     
void HMC5843::frequency(int hz)
{
    if (hz == 100000)
        i2c_->frequency(100000);
    else if (hz == 400000)
        i2c_->frequency(400000);

    return;
}
