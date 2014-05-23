#ifndef _SYSTEMSTATE_H
#define _SYSTEMSTATE_H

#define SSBUF 32 // must be 2^n

/** System State is the main mechanism for communicating current realtime system state to
 * the rest of the system for logging, data display, etc.
 */

#include <stdbool.h>

/* struct systemState
 * structure containing system sensor data
 ****** System Status
 * millis               number of milliseconds since epoch (or startup)
 * current              current draw in amps
 * voltage              voltage in volts
 ****** Data reported by IMU
 * g[3]                 raw 3-axis gyro values; if using 1-axis, then store data in gx
 * gTemp                Gyro temperature
 * a[3]                 raw 3-axis accelerometer values
 * m[3]                 raw 3-axis magnetometer values; if using 2d then store data in mx and my
 * gHeading             independently calculated gyro heading in degrees
 * cHeading             independently calculated compass heading in degrees
 ****** AHRS Estimates
 * roll, pitch, yaw     estimated attitude in degrees relative to the world frame
 ****** Data reported by GPS
 * gpsLatitude          raw GPS latitude in fractional degrees (e.g., 39.123456)
 * gpsLongitude         raw GPS longitude in fractional degrees (e.g., -104.123456
 * gpsCourse_deg        raw GPS course in degrees
 * gpsSpeed_mps         raw GPS speed in m/s
 * gpsHDOP              raw GPS Horizontal Dilution of Precision
 * gpsSats              raw GPS Satellite fix count
 ****** Odometry data
 * lrEncDistance        left rear encoder distance since last log update
 * rrEncDistance        right rear encoder distance since last log update
 * lrEncSpeed           left rear encoder speed 
 * rrEncSpeed           right rear encoder speed
 * encHeading           estimated heading based on encoder readings
 ****** Estimated Position and Heading
 * estLagHeading        estimated heading in degrees, lagged to sync with gps
 * estHeading           estimated current heading
 * estLatitude          estimated latitude in fractional degrees (e.g., 39.123456)
 * estLongitude         estimated longitude in fractional degrees (e.g., -104.123456)
 * estNorthing          some algorithms use UTM.  Estimated UTM northing
 * estEasting           estimated UTM easting
 * estX, estY           some algorithms use simple x, y distance from origin (meters)
 ****** Waypoint data
 * nextWaypoint         integer ID of the next waypoint
 * bearing              estimated bearing to next waypoint in degrees
 * distance             estimated distance to next waypoint in meters
 ****** Control data
 * throttle             raw servo setting(units?)
 * steering             raw servo setting(units?)
 */
typedef struct {
    unsigned int millis;
    float current, voltage;
    int g[3];
    float gyro[3];
    int gTemp;
    int a[3];
    int m[3];
    float gHeading;
    float cHeading;
    //float roll, pitch, yaw;
    double gpsLatitude;
    double gpsLongitude;
    float gpsCourse_deg;
    float gpsSpeed_mps;
    float gpsHDOP;
    int gpsSats;
    float lrEncDistance, rrEncDistance;
    float lrEncSpeed, rrEncSpeed;
    float encHeading;
    float estHeading;
    float estLagHeading;
    double estLatitude, estLongitude;
    //double estNorthing, estEasting;
    float estX, estY;
    unsigned short nextWaypoint;
    float bearing;
    float distance;
    float gbias;
    float errHeading;
    float steerAngle;
    float LABrg;
    float LAx;
    float LAy;
} SystemState;

void state_clear( SystemState *s );
bool fifo_init(void);
void fifo_reset(void);
bool fifo_available(void);
bool fifo_push(SystemState *s);
SystemState *fifo_first(void);
SystemState *fifo_last(void);
SystemState *fifo_pull(void);
int fifo_getInState(void);
int fifo_getOutState(void);

#endif
