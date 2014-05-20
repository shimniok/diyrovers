#define MAXBUF 64

#include "Config.h"
#include "SDFileSystem.h"
#include "GeoPosition.h"
#include "globals.h"
#include "util.h"

#define CONFIGFILE	"/etc/config.txt"

// Identifiers for each of the parameters
#define CURB 			"curb"
#define WAYPOINT		"wpt"
#define GPS				"gps"
#define GYRO			"gyro"
#define MAGNETOMETER	"mag"
#define ACCELEROMETER	"accel"
#define DECLINATION		"decl"
#define NAVIGATION		"nav"
#define STEER			"steer"
#define SPEED			"speed"
#define VEHICLE			"veh"

extern Serial pc;

// TODO: 3: mod waypoints to include speed after waypoint

Config::Config():
    loaded(false)
,	interceptDist(0.0)
,	waypointDist(0.0)
,	brakeDist(0.0)
,	wptCount(0)
,	escMin(0)
,	escZero(0)
,	escMax(0)
,	topSpeed(0.0)
,	turnSpeed(0.0)
,	startSpeed(0.0)
,	minRadius(0.0)
,	speedKp(0.0)
,	speedKi(0.0)
,	speedKd(0.0)
,	steerZero(0.0)
,	steerGain(0.0)
,	steerGainAngle(0.0)
,	curbThreshold(0.0)
,	curbGain(0.0)
,	gyroBias(0.0)
{
    // not much to do here, yet.
}

// load configuration from filesystem
bool Config::load()
{
    FILE *fp;
    char buf[MAXBUF];   // buffer to read in data
    char tmp[MAXBUF];   // temp buffer
    char *p;
    double lat, lon;
    float wTopSpeed, wTurnSpeed;
    bool declFound = false;
    bool confStatus = false;

    pc.printf("opening config file...\n");
    
    fp = fopen(CONFIGFILE, "r");
    if (fp == 0) {
        pc.puts("Could not open ");
        pc.puts(CONFIGFILE);
        pc.puts(" \n");
    } else {
        wptCount = 0;
        while (!feof(fp)) {
            fgets(buf, MAXBUF-1, fp);
            p = split(tmp, buf, MAXBUF, ',');           	// split off the first field

            if (!strcmp(tmp, CURB)) {

            	p = split(tmp, p, MAXBUF, ',');     		// threshold distance for curb avoid
				curbThreshold = cvstof(tmp);
				p = split(tmp, p, MAXBUF, ',');     		// steering gain for curb avoid
				curbGain = cvstof(tmp);

            } else if (!strcmp(tmp, WAYPOINT)) {

            	p = split(tmp, p, MAXBUF, ',');     		// split off the latitude to tmp
				lat = cvstof(tmp);
				p = split(tmp, p, MAXBUF, ',');     		// split off the longitude to tmp
				lon = cvstof(tmp);
				p = split(tmp, p, MAXBUF, ',');				// split off the waypoint top speed
				wTopSpeed = cvstof(tmp);
				p = split(tmp, p, MAXBUF, ',');				// split off the waypoint turn speed
				wTurnSpeed = cvstof(tmp);
				if (wptCount < MAXWPT) {
					wpt[wptCount].set(lat, lon);			// set position
					wptTopSpeedAdj[wptCount] = wTopSpeed;	// set top speed adjust
					wptTurnSpeedAdj[wptCount] = wTurnSpeed;	// set turn speed adjust
					wptCount++;
				}

            } else if (!strcmp(tmp, NAVIGATION)) {

            	p = split(tmp, p, MAXBUF, ',');
				interceptDist = (float) cvstof(tmp);    	// intercept distance for steering algorithm
				p = split(tmp, p, MAXBUF, ',');
				waypointDist = (float) cvstof(tmp);     	// distance before waypoint switch
				p = split(tmp, p, MAXBUF, ',');
				brakeDist = (float) cvstof(tmp);       		// distance at which braking starts
				p = split(tmp, p, MAXBUF, ',');
				minRadius = (float) cvstof(tmp);        	// minimum turning radius

            } else if (!strcmp(tmp, STEER)) {

            	p = split(tmp, p, MAXBUF, ',');
				steerZero = cvstof(tmp);            		// servo center setting
				p = split(tmp, p, MAXBUF, ',');
				steerGain = cvstof(tmp);            		// steering angle multiplier
				p = split(tmp, p, MAXBUF, ',');
				steerGainAngle = cvstof(tmp);       		// angle below which steering gain takes effect

            } else if (!strcmp(tmp, SPEED)) {

            	p = split(tmp, p, MAXBUF, ',');
				escMin = cvstof(tmp);                 		// minimum esc (brake) setting
				p = split(tmp, p, MAXBUF, ',');
				escZero = cvstof(tmp);                		// esc center/zero setting
				p = split(tmp, p, MAXBUF, ',');
				escMax = cvstof(tmp);                 		// maximum esc setting
				p = split(tmp, p, MAXBUF, ',');
				topSpeed = cvstof(tmp);             		// desired top speed
				p = split(tmp, p, MAXBUF, ',');
				turnSpeed = cvstof(tmp);            		// speed to use when turning
				p = split(tmp, p, MAXBUF, ',');
				startSpeed = cvstof(tmp);           		// speed to use at start
				p = split(tmp, p, MAXBUF, ',');
				speedKp = cvstof(tmp);              		// speed PID: proportional gain
				p = split(tmp, p, MAXBUF, ',');
				speedKi = cvstof(tmp);             			// speed PID: integral gain
				p = split(tmp, p, MAXBUF, ',');
				speedKd = cvstof(tmp);              		// speed PID: derivative gain

            } else if (!strcmp(tmp, VEHICLE)) {
            	p = split(tmp, p, MAXBUF, ',');
				wheelbase = cvstof(tmp);              		// vehicle wheelbase (front to rear axle)
				p = split(tmp, p, MAXBUF, ',');
				track = cvstof(tmp);                		// vehicle track width (left to right)
            } //if-else
            /*
			} else if (!strcmp(tmp, GYRO)) {
				p = split(tmp, p, MAXBUF, ',');     // split off the declination to tmp
				gyroBias = (float) cvstof(tmp);
			} else if (!strcmp(tmp, DECLINATION)) {
				p = split(tmp, p, MAXBUF, ',');     // split off the declination to tmp
				declination = (float) cvstof(tmp);
				declFound = true;
			} else if (!strcmp(tmp, MAGNETOMETER)) {
				for (int i=0; i < 3; i++) {
					p = split(tmp, p, MAXBUF, ',');
					magOffset[i] = (float) cvstof(tmp);
					pc.printf("magOffset[%d]: %.2f\n", i, magOffset[i]);
				}
				for (int i=0; i < 3; i++) {
					p = split(tmp, p, MAXBUF, ',');
					magScale[i] = (float) cvstof(tmp);
					pc.printf("magScale[%d]: %.2f\n", i, magScale[i]);
				}
			}
            */
        } // while

        // Did we get the values we were looking for?
        if (wptCount > 0 && declFound) {
            confStatus = true;
        }
        
    } // if fp
    
    if (fp != 0)
        fclose(fp);

    loaded = confStatus;

    return confStatus;
}
