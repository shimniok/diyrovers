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
#ifndef __SENSORS_H
#define __SENSORS_H

/** Sensor interface library abstracts sensor drivers, next step to a pluggable architecture */

#include "Sirf3.h"
#include "Ublox6.h"
#include "Venus638flpx.h"
#include "L3G4200D.h"
#include "LSM303DLM.h"
//#include "HMC5843.h"
#include "IncrementalEncoder.h"
#include "Matrix.h"

// Sensor axes
#define _x_ 0
#define _y_ 1
#define _z_ 2

// Magnetometer calibration constants
//#define M_OFFSET_X -20.4416 // calibrated 02/13/2012 mes
//#define M_OFFSET_Y -67.2318
//#define M_OFFSET_Z 6.0950
//#define M_OFFSET_X 12.8922 // calibrated 12/31/2012 mes
//#define M_OFFSET_Y -7.3453
//#define M_OFFSET_Z -147.8652
//#define M_SCALE_X 570.06
//#define M_SCALE_Y 532.83
//#define M_SCALE_Z 480.95

//#define M_X_MIN -654 // calibrated 12/8/2011 mes
//#define M_X_MAX 457
//#define M_Y_MIN -744 
//#define M_Y_MAX 369
//#define M_Z_MIN -573
//#define M_Z_MAX 464

// Chassis specific parameters
#define WHEEL_STRIPES 32
#define WHEEL_CIRC    0.321537 // m; calibrated with 4 12.236m runs. Wheel circumference measured 13.125" or 0.333375m
#define WHEELBASE     0.290
#define TRACK         0.280

class Sensors {
public:
    Sensors(void);
    void Compass_Calibrate(float offset[3], float scale[3]);
    void Read_Encoders(void);
    void Read_Gyro(void);
    void Read_Accel(void);
    void Read_Compass(void);
    void Calculate_Offsets(void);
    void Compass_Heading(void);
    void getRawMag(int mag[3]);
    float getVoltage(void);
    float getCurrent(void);
    void Read_Power(void);
    void Read_Rangers();
    void Read_Camera();
    int g[3];                               // raw gyro value
    int gTemp;                              // raw gyro temperature
    int a[3];                               // raw accelerometer value
    int m[3];                               // raw magnetometer value
    int g_offset[3];                        // raw gyro offset
    int a_offset[3];                        // raw accel offset
    float m_offset[3];                      // magnetometer offset
    float g_scale[3];                       // gyro scaling factor
    float m_scale[3];                       // magnetometer scale
    int g_sign[3];                          // correct sensor signs
    int a_sign[3];
    int m_sign[3]; 
    float gyro[3];                          // corrected gyro value
    float accel[3];                         // corrected accelerometer value
    float mag[3];                           // corrected magnetometer value
    int ranger[3];                          // ranger values
    float leftRanger;
    float rightRanger;
    float centerRanger;
    float voltage;                          // battery voltage in volts
    float current;                          // system current draw in amps
    unsigned int leftTotal;                 // total number of ticks
    unsigned int rightTotal;                // total number of ticks
    unsigned int leftCount;                 // left rear encoder count
    unsigned int rightCount;                // right rear encoder count
    float lrEncDistance;                    // left rear encoder distance
    float rrEncDistance;                    // right rear encoder distance
    float lrEncSpeed;                       // left rear encoder speed
    float rrEncSpeed;                       // right rear encoder speed
    float encDistance;                      // encoder distance since last check
    float encSpeed;                         // encoder calculated speed

    //Sirf3 gps;                              // Pharos SiRF III GPS
    //Venus638flpx gps;                       // Venus GPS
    Ublox6 gps;                             // Ublox6 GPS
    AnalogIn _voltage;                      // Voltage from sensor board
    AnalogIn _current;                      // Current from sensor board
    IncrementalEncoder _left;               // Left wheel encoder
    IncrementalEncoder _right;              // Right wheel encoder
    L3G4200D _gyro;                         // MinIMU-9 gyro
    LSM303DLM _compass;                     // MinIMU-9 compass/accelerometer
    I2C _rangers;                           // Arduino ranger board
    I2C _cam;                               // Propeller camer aboard
    //HMC5843 compass;

private:
    void BubbleSort(float *num, int numLength);
};

#endif