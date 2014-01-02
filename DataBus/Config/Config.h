#ifndef __CONFIG_H
#define __CONFIG_H

#include "GeoPosition.h"
#include "CartPosition.h"

/** Text-based configuration; reads config file and stores in fields
 */
class Config {
    public:
        Config();

        bool load();
        bool loaded;            // has the config been loaded yet?
        float interceptDist;    // used for course correction steering calculation
        float waypointDist;     // distance threshold to waypoint
        float brakeDist;        // braking distance
        float declination;
        float compassGain;      // probably don't need this anymore
        float yawGain;          // probably don't need this anymore
        GeoPosition wpt[10];    // Waypoints, lat/lon coords
        CartPosition cwpt[10];  // Waypoints, cartesian coords
        unsigned int wptCount;  // number of active waypoints
        int escMin;             // minimum ESC value; brake
        int escZero;            // zero throttle
        int escMax;             // max throttle
        float topSpeed;         // top speed to achieve on the straights
        float turnSpeed;        // speed for turns
        float startSpeed;       // speed for start 
        float minRadius;        // minimum turning radius (calculated)
        float speedKp;          // Speed PID proportional gain
        float speedKi;          // Speed PID integral gain
        float speedKd;          // Speed PID derivative gain
        float steerZero;        // zero steering aka center point
        float steerGain;        // gain factor for steering algorithm
        float steerGainAngle;   // angle below which steering gain takes effect
        float curbThreshold;    // distance at which curb avoid takes place
        float curbGain;         // gain of curb avoid steering
        // acceleration profile?
        // braking profile?
        float gyroBias;     // this needs to be 3d
        // float gyroScale[3];
        float magOffset[3];
        float magScale[3];
        // float accelOffset[3];
        // float accelScale[3];
        // int gpsType;
        // int gpsBaud;
        // int gpsLag;
};

#endif
