/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

#include <stdio.h>
#include "devices.h"
#include "Sensors.h"
#include "debug.h"

#define GYRO_SCALE 14.49787 // Is the sign right here?? yes, see g_sign

#define VFF 50.0 // voltage filter factor

///////////////////// MAGNETOMETER CALIBRATION

Sensors::Sensors():
	gTemp(0),
	leftRanger(0),
	rightRanger(0),
	centerRanger(0),
	voltage(0),
	current(0),
	leftTotal(0),
	rightTotal(0),
	leftCount(0),
	rightCount(0),
	lrEncDistance(0.0),
	rrEncDistance(0.0),
	lrEncSpeed(0.0),
	rrEncSpeed(0.0),
	encDistance(0.0),
	encSpeed(0.0),
    gps(GPSTX, GPSRX),
    _voltage(p19),               // Voltage from sensor board
    _current(p20),               // Current from sensor board
    _left(ENCALEFT),                  // left wheel encoder
    _right(ENCARIGHT),                 // Right wheel encoder
    _gyro(I2CSDA, I2CSCL),       // MinIMU-9 gyro
    _compass(I2CSDA, I2CSCL),    // MinIMU-9 compass/accelerometer
    _rangers(I2CSDA, I2CSCL),    // Arduino ADC to I2C
    _cam(I2CSDA, I2CSCL)
{
    for (int i=0; i < 3; i++) {
        m_offset[i] = 0;
        m_scale[i] = 1;
    }

    // TODO 2 parameterize scale and sign for mag, accel, gyro
    g_scale[0] = 1;
    g_scale[1] = 1;
    g_scale[2] = GYRO_SCALE;

    g_sign[0] = 1;
    g_sign[1] = -1;
    g_sign[2] = -1;

    a_sign[0] = -1;
    a_sign[1] = 1;
    a_sign[2] = 1;

    m_sign[0] = 1;
    m_sign[1] = -1;
    m_sign[2] = -1;

    // upside down mounting
    //g_sign[3] = {1,1,1};        
    //a_sign[3] = {-1,-1,-1};
    //m_sign[3] = {1,1,1}; 
}

/* Compass_Calibrate
 *
 * Set the offset and scale calibration for the compass
 */
void Sensors::Compass_Calibrate(float offset[3], float scale[3])
{
    for (int i=0; i < 3; i++) {
        m_offset[i] = offset[i];
        m_scale[i] = scale[i];
    }    
 
    return;
}


void Sensors::Read_Encoders()
{
    // Odometry            
    leftCount = _left.read();
    rightCount = _right.read();
    leftTotal += leftCount;
    rightTotal += rightCount;
            
    // TODO 2 sanity check on encoders; if difference between them
    //  is huge, what do we do?  Slipping wheel?  Skidding wheel?  Broken encoder?
    //  front encoders would be ideal as additional sanity check
    
    // TODO 2 move Read_Encoders() into scheduler??
    
    // TODO 2 how do we track distance, should we only check distance everytime we do a nav/pos update?
    // TODO 3 get rid of state variable
    lrEncDistance  = (WHEEL_CIRC / WHEEL_STRIPES) * (double) leftCount;
    rrEncDistance = (WHEEL_CIRC / WHEEL_STRIPES) * (double) rightCount;
    //encDistance = (lrEncDistance + rrEncDistance) / 2.0;
    // compute speed from time between ticks
    int leftTime = _left.readTime();
    int rightTime = _right.readTime();

    // We could also supplement this with distance/time calcs once we get up to a higher
    // speed and encoder quantization error is lower
    // To reduce asymmetry of the encoder operation, we're only reading time
    // between rising ticks. So that means half the wheel stripes

    if (leftTime <= 0) {
        lrEncSpeed = 0;
    } else {
        lrEncSpeed = (2.0 * WHEEL_CIRC / WHEEL_STRIPES) / ((float) leftTime * 1e-6);
    }
    
    if (rightTime <= 0) {
        rrEncSpeed = 0;
    } else {
        rrEncSpeed = (2.0 * WHEEL_CIRC / WHEEL_STRIPES) / ((float) rightTime * 1e-6);
    }
        
    // Dead band
    if (lrEncSpeed < 0.1) lrEncSpeed = 0;
    if (rrEncSpeed < 0.1) rrEncSpeed = 0;
    // TODO: 3 use more sophisticated approach where we count if any ticks have happened lately
    // and if not, reset the speed
    encSpeed = (lrEncSpeed + rrEncSpeed) / 2.0;
}


void Sensors::Read_Gyro()
{
    _gyro.read(g);
    gTemp = (int) _gyro.readTemp();
    
    gyro[_x_] = g_sign[_x_] * (g[_x_] - g_offset[_x_]); // really need to scale this too
    gyro[_y_] = g_sign[_y_] * (g[_y_] - g_offset[_y_]); // really need to scale this too
    gyro[_z_] = (g_sign[_z_] * (g[_z_] - g_offset[_z_])) / g_scale[_z_];

    return;
}

// Reads x,y and z accelerometer registers
void Sensors::Read_Accel()
{
    _compass.readAcc(a);

    accel[_x_] = a_sign[_x_] * (a[_x_] - a_offset[_x_]);
    accel[_y_] = a_sign[_y_] * (a[_y_] - a_offset[_y_]);
    accel[_z_] = a_sign[_z_] * (a[_z_] - a_offset[_z_]);

    return;  
}

void Sensors::Read_Compass()
{
    _compass.readMag(m);
    // adjust for compass axis offsets, scale, and orientation (sign)
    for (int i=0; i < 3; i++) {
        mag[i] = ((float) m[i] - m_offset[i]) * m_scale[i] * m_sign[i];
    }
    //fprintf(stdout, "m_offset=(%5.5f,%5.5f,%5.5f)\n", ahrs.c_magnetom[0],ahrs.c_magnetom[1],ahrs.c_magnetom[2]);
  
    return;
}

/* doing some crude gryo and accelerometer offset calibration here
 *
 */
void Sensors::Calculate_Offsets()
{
    int samples=128;
    
    for (int i=0; i < 3; i++) {
        g_offset[i] = 0;
        a_offset[i] = 0;
    }

    for(int i=0; i < samples; i++) {  // We take some readings...
        Read_Gyro();
        Read_Accel();
        wait(0.010); // sample at 100hz
        for(int y=0; y < 3; y++) {   // accumulate values
            g_offset[y] += g[y];
            a_offset[y] += a[y];
        }
    }

    for(int y=0; y < 3; y++) {
        g_offset[y] /= samples;
        a_offset[y] /= samples;
    }

    //TODO 4 if we ever get back to using accelerometer, do something about this.
    //a_offset[_z_] -= GRAVITY * a_sign[_z_]; // I don't get why we're doing this...?
}



void Sensors::Compass_Heading()
{
  /*
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  // See DCM.cpp Drift_correction()

  // umm... doesn't the dcm already have this info in it?!
  cos_roll = cos(ahrs.roll);
  sin_roll = sin(ahrs.roll);
  cos_pitch = cos(ahrs.pitch);
  sin_pitch = sin(ahrs.pitch);
    
  // Tilt compensated Magnetic filed X:
  MAG_X = mag[_x_] * cos_pitch + mag[_y_] * sin_roll * sin_pitch + mag[_z_] * cos_roll * sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = mag[_y_] * cos_roll - mag[_z_] * sin_roll;
  // Magnetic Heading
  ahrs.MAG_Heading = atan2(-MAG_Y,MAG_X);
  */
}


void Sensors::getRawMag(int rawmag[3])
{
    Read_Compass();
    for (int i=0; i < 3; i++) {
        rawmag[i] = m[i];
    }        
}


// getCurrent is a macro defined above

float Sensors::getVoltage() {
    static float filter = -1.0;
    float v = _voltage * 3.3 * 4.12712;        // 242.3mV/V

    // TODO 3 median filter to eliminate spikes
    if (filter < 0) {
        filter = v * VFF;
    } else {
        filter += (v - filter/VFF);
    }
    return (filter / VFF);
}



float Sensors::getCurrent() {
    /*static bool first=true;
    static float history[3];
    float tmp;
    float sort[3];

    if (first) {
        first = false;
        history[0] = history[1] = history[2] = _current;
    } else {
        sort[0] = history[0] = history[1];
        sort[1] = history[1] = history[2];
        sort[2] = history[2] = _current;
    }
    BubbleSort(sort, 3);*/
    return (_current * 3.3 * 13.6612); // 73.20mV/A        
}


void Sensors::Read_Power()
{
    // TODO 3 exponential or median filtering on these to get rid of spikes
    //
    voltage = getVoltage();
    current = getCurrent();

    return;
}

float clampIRRanger(float x)
{
    float y=x;
    
    if (y < 0.1) 
        y = 0.1;
    if (y > 5.0) 
        y = 5.0;
    
    return y;
}

void Sensors::Read_Camera()
{
    char count;
    //char data[128];
    //char addr=0x80;
    /*
    struct {
        int color;
        int x1;
        int y1;
        int x2;
        int y2;
    } blob;
    */

    /* 
    I2C START BIT
    WRITE: 0xA0 ACK 
    WRITE: 0x00 ACK 
    I2C START BIT
    WRITE: 0xA1 ACK 
    READ: 0x01  ACK 0x02  ACK 0x03  ACK 0x04  ACK 0x05  ACK 0x06  ACK 0x07  ACK 0x08  ACK 0x09  ACK 0x0A 
    NACK
    I2C STOP BIT
    */

    _cam.start();
    _cam.write(0x11<<1);
    _cam.write(0x01); // command to send box data
    _cam.start();
    _cam.write((0x11<<1 | 0x01));
    count = _cam.read(1); // number of boxes tracked
    if (count > 8) count = 8;
    //fprintf(stdout, "----------\n%d\n", count);
    int x=0;
    for (int i=0; i < count; i++) {
        //fprintf(stdout, "%d: ", i);
        for (int j=0; j < 5; j++) {
            //data[x] = _cam.read(1);
            //fprintf(stdout, "%d ", data[x]);
            x++;
        }
        //fprintf(stdout, "\n");
    }
    _cam.read(0); // easiest to just read one more byte then NACK it
    _cam.stop();
}


void Sensors::Read_Rangers()
{
    char data[2];
    // Sharp GP2Y0A710YK0F
    static float m=288.12;  // slope adc=m(dist) + b
    static float b=219.3;
    static float m_per_lsb=0.004883/0.38583; // 0.004883mV/lsb * 0.0098in/lsb * 1in/0.0254m = m/lsb
    
    _rangers.start();
    data[0] = (0x11<<1 | 0x01); // send address + read = 1
    _rangers.write(data[0]);       // send address
    ranger[0] = (int) _rangers.read(1) | ((int) _rangers.read(1)) << 8;
    ranger[1] = (int) _rangers.read(1) | ((int) _rangers.read(1)) << 8;
    ranger[2] = (int) _rangers.read(1) | ((int) _rangers.read(0)) << 8; // don't ack the last byte
    _rangers.stop();

    /*
    for (int q=0; q<3; q++)
        fprintf(stdout, "%04x ", ranger[q]);    
    fprintf(stdout, "\n");
    */
    
    if (ranger[0] < 86) {
        rightRanger = 999.0;
    } else if (ranger[0] > 585) {
        rightRanger = 20.0;
    } else {
        // Sharp GP2Y0A02YK0F
        rightRanger = 1/(8.664e-005*ranger[0]-0.0008); // IR distance, meters
    }

    // Compute distances
    leftRanger = clampIRRanger( m/(ranger[1]-b) );  // IR distance, meters
    // Sonar
    centerRanger = ranger[2] * (m_per_lsb); // Sonar distance, metersmv/in=0.0098, mv/lsb=0.0049
}



/** perform bubble sort
 * for median filtering
 */
void Sensors::BubbleSort(float *num, int numLength)
{
    float temp;             // holding variable
    bool flag=true;
    
    for(int i = 1; (i <= numLength) && flag; i++) {
        flag = false;
        for (int j = 0; j < (numLength -1); j++) {
            if (num[j+1] > num[j]) {      // ascending order simply changes to <
                temp = num[j];             // swap elements
                num[j] = num[j+1];
                num[j+1] = temp;
                flag = true;
            }
        }
    }
    return;   //arrays are passed to functions by address; nothing is returned
}
