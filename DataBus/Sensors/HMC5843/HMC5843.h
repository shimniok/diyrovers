/**
 * @author Uwe Gartmann
 * @author Used HMC5843 library developed by Jose R. Padron and Aaron Berk as template
 *
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
 * Honeywell HMC5843 digital compass.
 *
 * Datasheet:
 *
 * http://www.ssec.honeywell.com/magnetic/datasheets/HMC5843.pdf
 */

#ifndef HMC5843_H
#define HMC5843_H

#include "mbed.h"

#define HMC5843_I2C_ADDRESS 0x1E //7-bit address. 0x3C write, 0x3D read.
#define HMC5843_I2C_WRITE   0x3C 
#define HMC5843_I2C_READ    0x3D 

//Values Config A
#define HMC5843_0_5HZ_NORMAL         0x00
#define HMC5843_0_5HZ_POSITIVE       0x01
#define HMC5843_0_5HZ_NEGATIVE       0x02

#define HMC5843_1HZ_NORMAL           0x04
#define HMC5843_1HZ_POSITIVE         0x05
#define HMC5843_1HZ_NEGATIVE         0x06

#define HMC5843_2HZ_NORMAL           0x08
#define HMC5843_2HZ_POSITIVE         0x09
#define HMC5843_2HZ_NEGATIVE         0x0A

#define HMC5843_5HZ_NORMAL           0x0C
#define HMC5843_5HZ_POSITIVE         0x0D
#define HMC5843_5HZ_NEGATIVE         0x0E

#define HMC5843_10HZ_NORMAL           0x10
#define HMC5843_10HZ_POSITIVE         0x11
#define HMC5843_10HZ_NEGATIVE         0x12

#define HMC5843_20HZ_NORMAL           0x14
#define HMC5843_20HZ_POSITIVE         0x15
#define HMC5843_20HZ_NEGATIVE         0x16

#define HMC5843_50HZ_NORMAL           0x18
#define HMC5843_50HZ_POSITIVE         0x19
#define HMC5843_50HZ_NEGATIVE         0x1A

//Values Config B
#define HMC5843_0_7GA         0x00
#define HMC5843_1_0GA         0x20
#define HMC5843_1_5GA         0x40
#define HMC5843_2_0GA         0x60
#define HMC5843_3_2GA         0x80
#define HMC5843_3_8GA         0xA0
#define HMC5843_4_5GA         0xC0
#define HMC5843_6_5GA         0xE0

//Values MODE
#define HMC5843_CONTINUOUS   0x00
#define HMC5843_SINGLE       0x01
#define HMC5843_IDLE         0x02
#define HMC5843_SLEEP        0x03



#define HMC5843_CONFIG_A     0x00
#define HMC5843_CONFIG_B     0x01
#define HMC5843_MODE         0x02
#define HMC5843_X_MSB        0x03
#define HMC5843_X_LSB        0x04
#define HMC5843_Y_MSB        0x05
#define HMC5843_Y_LSB        0x06
#define HMC5843_Z_MSB        0x07
#define HMC5843_Z_LSB        0x08
#define HMC5843_STATUS       0x09
#define HMC5843_IDENT_A      0x0A
#define HMC5843_IDENT_B      0x0B
#define HMC5843_IDENT_C      0x0C



/**
 * Interface library for the Honeywell HMC5843 digital compass.
 * @author Michael Shimniok http://www.bot-thoughts.com/
 * @author Based on library by Uwe Gartmann
 * @author In turn based on HMC5843 library developed by Jose R. Padron and Aaron Berk as template
 *
 * This version is modified to follow the same member functions of my LSM303DLH library. The intent
 * is to make it drop in compatible with my other magnetometer libraries.  For now, setScale() and
 * setOffset() have no effect at this time.
 *
 * @code
 * #include "mbed.h"
 * #include "HMC5843.h"
 *
 * HMC5843 compass(p28, p27);
 * ...
 * int m[3];
 * ...
 * compass.readMag(m);
 *
 * @endcode
 */
class HMC5843 {

public:

    /**
     * Create a new interface for an HMC5843.
     *
     * @param sda mbed pin to use for SDA line of I2C interface.
     * @param scl mbed pin to use for SCL line of I2C interface.
     */
    HMC5843(PinName sda, PinName scl);


   /** sets the x, y, and z offset corrections for hard iron calibration
    * 
    * Calibration details here:
    *  http://mbed.org/users/shimniok/notebook/quick-and-dirty-3d-compass-calibration/
    *
    * If you gather raw magnetometer data and find, for example, x is offset
    * by hard iron by -20 then pass +20 to this member function to correct
    * for hard iron.
    *
    * @param x is the offset correction for the x axis
    * @param y is the offset correction for the y axis
    * @param z is the offset correction for the z axis
    */
    void setOffset(float x, float y, float z);


    /** sets the scale factor for the x, y, and z axes
     *
     * Calibratio details here:
     *  http://mbed.org/users/shimniok/notebook/quick-and-dirty-3d-compass-calibration/
     *
     * Sensitivity of the three axes is never perfectly identical and this
     * function can help to correct differences in sensitivity.  You're
     * supplying a multipler such that x, y and z will be normalized to the
     * same max/min values
     */
    void setScale(float x, float y, float z);

        
    /**
     * Enter into sleep mode.
     */
    void setSleepMode();
    
       
    /**
     * Set Device in Default Mode.
     * HMC5843_CONTINUOUS, HMC5843_10HZ_NORMAL HMC5843_1_0GA
     */
    void setDefault();

       
    /**
     * Set the operation mode.
     *
     * @param mode 0x00 -> Continuous
     *             0x01 -> Single
     *             0x02 -> Idle
     * @param ConfigA values
     * @param ConfigB values
     */
    void setOpMode(int mode, int ConfigA, int ConfigB);

    
    /**
     * Write to a register on the device.
     *
     * @param address is the register address to be written.
     * @param data is the data to write.
     */
    void write(int address, int data);


    /** read the calibrated accelerometer and magnetometer values
     *
     * @param m is the magnetometer 3d vector, written by the function
     */
     void read(int m[3]);


     /** read the calibrated magnetometer values
      *
      * @param m is the magnetometer 3d vector, written by the function
      */
     void readMag(int m[3]);


     /**
     * Get the output of all three axes.
     *
     * @param Pointer to a buffer to hold the magnetics value for the
     *        x-axis, y-axis and z-axis [in that order].
     */
    void readData(int* readings);

    
    /**
     * Get the output of X axis.
     *
     * @return x-axis magnetic value
     */
    int getMx();

    
    /**
     * Get the output of Y axis.
     *
     * @return y-axis magnetic value
     */
    int getMy();

    
    /**
     * Get the output of Z axis.
     *
     * @return z-axis magnetic value
     */
    int getMz();
   
    /**
     * Get the current operation mode.
     *
     * @return Status register values
     */
    int getStatus(void);


    /**
     * Read the memory location on the device which contains the address.
     *
     * @param Pointer to a buffer to hold the address value
     * Expected     H, 4 and 3.
     */
    void getAddress(char * address);

    
    /** sets the I2C bus frequency
     *
     * @param frequency is the I2C bus/clock frequency, either standard (100000) or fast (400000)
     */
    void frequency(int hz);

private:
    I2C* i2c_;

};

#endif /* HMC5843_H */
