#ifndef __CONFIG_H
#define __CONFIG_H

#include "GeoPosition.h"
#include "CartPosition.h"

/** Text-based configuration; reads config file and stores in fields
 */
class Config {
    public:
		static const int MAX_WPT=10;	// maximum number of waypoints
        Config();
        bool load();
        bool loaded;            // has the config been loaded yet?
        float interceptDist;    // used for course correction steering calculation
        float waypointDist;     	// distance threshold to waypoint
        float brakeDist;        	// braking distance
        GeoPosition wpt[MAX_WPT];   // Waypoints, lat/lon coords
        CartPosition cwpt[MAX_WPT]; // Waypoints, cartesian coords
        float wptTopSpeedAdj[MAX_WPT];	// Speed approaching waypoint
        float wptTurnSpeedAdj[MAX_WPT];
        unsigned int wptCount;  	// number of active waypoints
        float escMin;             	// minimum ESC value; brake
        float escZero;            	// zero throttle
        float escMax;             	// max throttle
        float topSpeed;         	// default top speed to achieve on the straights
        float turnSpeed;        	// default speed for turns
        float startSpeed;       	// speed for start
        float minRadius;        	// minimum turning radius (calculated)
        float speedKp;          	// Speed PID proportional gain
        float speedKi;          	// Speed PID integral gain
        float speedKd;          	// Speed PID derivative gain
        float steerZero;        	// zero steering aka center point
        float steerGain;        	// gain factor for steering algorithm
        float steerGainAngle;   	// angle below which steering gain takes effect
        float curbThreshold;    	// distance at which curb avoid takes place
        float curbGain;        		// gain of curb avoid steering
        float gyroBias;     		// this needs to be 3d
        // float gyroScale[3];
        float magOffset[3];
        float magScale[3];
        // float accelOffset[3];
        // float accelScale[3];
        float wheelbase;			// Vehicle wheelbase, front to rear axle
        float track;				// Vehicle track width, left to right contact patch
        float tireCirc;				// tire circumference
        int encStripes;				// Number of ticks per revolution
};

#endif
