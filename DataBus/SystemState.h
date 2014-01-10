#ifndef _SYSTEMSTATE_H
#define _SYSTEMSTATE_H

#define SSBUF 8 // must be 2^n

/** System State is the main mechanism for communicating current realtime system state to
 * the rest of the system for logging, data display, etc.
 */

#include <stdbool.h>

/* struct to hold system state, all relevant variables */
typedef struct {
    unsigned int millis;		// number of milliseconds since epoch (or startup)
    float current;				// current draw in amps
    float voltage;				// voltage in volts
    int g[3];					// raw 3-axis gyro values
    float gyro[3];				// corrected, signed, scaled gyro (calculated)
    int gTemp;					// gyro temperature
    int a[3];					// raw 3-axis accelerometer values
    int m[3];					// raw 3-axis magnetometer values
    float gHeading;				// independently calculated gyro heading in degrees
    float cHeading;				// independently calculated compass heading in degrees
    //float roll, pitch, yaw;
    double gpsLatitude;			// GPS latitude in fractional degrees (e.g., 39.123456)
    double gpsLongitude;		// GPS longitude in fractional degrees (e.g., -104.123456
    float gpsCourse_deg;		// GPS course in degrees
    float gpsSpeed_mps;			// GPS speed in m/s
    float gpsHDOP;				// GPS Horizontal Dilution of Precision
    int gpsSats;				// GPS Satellite fix count
    float lrEncDistance;		// left rear encoder distance (calculated)
    float rrEncDistance;		// right rear encoder distance_m (calculated)
    float lrEncSpeed;			// left rear encoder speed (calculated)
    float rrEncSpeed;			// right rear encoder speed (calculated)
    float encHeading;			// estimated heading based on encoder readings
    float estHeading;			// estimated current heading
    float estLagHeading;		// estimated heading in degrees, lagged to sync with gps
    double estLatitude;			// estimated latitude in fractional degrees (e.g., 39.123456)
    double estLongitude;		// estimated longitude in fractional degrees (e.g., -104.123456)
    float estX;					// estimated x (m) from origin
    float estY;					// estimated y (m) from origin
    unsigned short nextWaypoint;// integer ID of the next waypoint
    float bearing_deg;			// estimated bearing to next waypoint in degrees
    float distance_m;			// estimated distance_m to next waypoint in meters
    float gbias;				// estimated gyro bias
    float errHeading;			// estimated gyro-computed heading error
    float steerAngle;			// computed steering angle
} SystemState;

void state_clear( SystemState *s );
bool fifo_init(void);
void fifo_reset(void);
bool fifo_available(void);
bool fifo_push(SystemState *s);
SystemState *fifo_first(void);
SystemState *fifo_last(void);
SystemState *fifo_pull(void);

#endif
