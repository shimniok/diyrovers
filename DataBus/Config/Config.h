#ifndef __CONFIG_H
#define __CONFIG_H

#include "GeoPosition.h"
#include "CartPosition.h"

/** Text-based configuration; reads config file and stores in fields
 */
class Config {
    public:
		static const int MAX_WPT=10;			// maximum number of waypoints

		/** Create a new config instance */
		Config();

		/** Load configuration from file */
        bool load(void);

        static bool loaded;            			// has the config been loaded yet?
        static float intercept;    				// used for course correction steering calculation
        static float waypointDist;	     		// distance threshold to waypoint
        static float brakeDist;   	     		// braking distance
        static GeoPosition wpt[MAX_WPT]; 		// Waypoints, lat/lon coords
        static CartPosition cwpt[MAX_WPT];		// Waypoints, cartesian coords
        static float wptTopSpeedAdj[MAX_WPT];	// Speed approaching waypoint
        static float wptTurnSpeedAdj[MAX_WPT];
        static unsigned int wptCount;		  	// number of active waypoints
        static float escMin;             		// minimum ESC value; brake
        static float escZero;            		// zero throttle
        static float escMax;           		  	// max throttle
        static float topSpeed;	      		   	// default top speed to achieve on the straights
        static float turnSpeed;	        		// default speed for turns
        static float startSpeed;		       	// speed for start
        static float minRadius;	        		// minimum turning radius (calculated)
        static float speedKp;		          	// Speed PID proportional gain
        static float speedKi;		          	// Speed PID integral gain
        static float speedKd;		          	// Speed PID derivative gain
        static float steerZero;        			// zero steering aka center point
        static float steerScale;     	   		// gain factor for steering algorithm
        static float curbThreshold; 	   		// distance at which curb avoid takes place
        static float curbGain;      	  		// gain of curb avoid steering
//        static float gyroBias; 	    			// this needs to be 3d
        static float gyroScale;					// scales gyro output to degrees / sec
        // float gyroScale[3];
        static float magOffset[3];
        static float magScale[3];
        // float accelOffset[3];
        // float accelScale[3];
        static float wheelbase;					// Vehicle wheelbase, front to rear axle
        static float track;						// Vehicle track width, left to right contact patch
        static float tireCirc;					// tire circumference
        static int encStripes;					// Number of ticks per revolution
};

#endif
