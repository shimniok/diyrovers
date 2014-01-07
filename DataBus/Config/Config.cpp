#define MAXBUF 64

#include "Config.h"
#include "print.h"
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
#define ENCODER			"enc"

extern Serial pc;

// TODO: 3: mod waypoints to include speed after waypoint

Config::Config():
    loaded(false)
,	interceptDist(0.0)
,	waypointDist(0.0)
,	brakeDist(0.0)
,	wptCount(0)
,	tireCircum(0.0)
,	stripeCount(0)
,	wheelbase(0.0)
,	trackWidth(0.0)
,	escMin(0.0)
,	escZero(0.0)
,	escMax(0.0)
,	escScale(0.0)
,	topSpeed(0.0)
,	turnSpeed(0.0)
,	startSpeed(0.0)
,	minRadius(0.0)
,	speedKp(0.0)
,	speedKi(0.0)
,	speedKd(0.0)
,	steerZero(0.0)
,	steerScale(0.0)
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
    bool confStatus = false;

    fputs("opening config file...\n", stdout);
    
    fp = fopen(CONFIGFILE, "r");
    if (fp == 0) {
        fputs("Could not open", stdout);
        fputs(CONFIGFILE, stdout);
        fputc('\n', stdout);
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

            } else if (!strcmp(tmp, VEHICLE)) {

				p = split(tmp, p, MAXBUF, ',');
				wheelbase = (float) cvstof(tmp); 	    	// distance between front and rear axle
				p = split(tmp, p, MAXBUF, ',');
				trackWidth = (float) cvstof(tmp);       	// distance between left and right tires

            } else if (!strcmp(tmp, ENCODER)) {

            	p = split(tmp, p, MAXBUF, ',');
				tireCircum = (float) cvstof(tmp);  			// tire circumference
				p = split(tmp, p, MAXBUF, ',');
				stripeCount = atoi(tmp);       				// number of black and white wheel encoder stripes

            } else if (!strcmp(tmp, STEER)) {

            	p = split(tmp, p, MAXBUF, ',');
				steerZero = cvstof(tmp);            		// servo center setting
				p = split(tmp, p, MAXBUF, ',');
				steerScale = cvstof(tmp);       			// ratio of servo to steering angle

            } else if (!strcmp(tmp, SPEED)) {

            	p = split(tmp, p, MAXBUF, ',');
				escMin = cvstof(tmp);                 		// minimum esc (brake) setting
				p = split(tmp, p, MAXBUF, ',');
				escZero = cvstof(tmp);                		// esc center/zero setting
				p = split(tmp, p, MAXBUF, ',');
				escMax = cvstof(tmp);                 		// maximum esc setting
				p = split(tmp, p, MAXBUF, ',');
				escScale = cvstof(tmp);                		// ratio of servo to throttle value
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

            }//if-else
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
        if (wptCount > 0) {
            confStatus = true;
        }
        
    } // if fp
    
    if (fp != 0)
        fclose(fp);

    loaded = confStatus;

    return confStatus;
}

void Config::print(void) {

	// Print out vehicle configuration data
	fputs("Vehicle:", stdout);
	fputs("\n wheelbase=", stdout);
	printFloat(stdout, wheelbase, 3);
	fputs("\n track width=", stdout);
	printFloat(stdout, trackWidth, 3);
	fputc('\n', stdout);

	// Print out encoder configuration
	fputs("Encoders:", stdout);
	fputs("\n tire circum=", stdout);
	printFloat(stdout, tireCircum, 3);
	fputs("\n enc stripes=", stdout);
	printInt(stdout, stripeCount);
    fputc('\n', stdout);

    // Print out speed configuration data
    fputs("Speed:", stdout);
	fputs("\n escMin=", stdout);
	printFloat(stdout, escMin, 3);
	fputs("\n escZero=", stdout);
	printFloat(stdout, escZero, 3);
    fputs("\n escMax=", stdout);
    printFloat(stdout, escMax, 3);
    fputs("\n escScale=", stdout);
	printFloat(stdout, escScale, 4);
	fputs("\n Top=", stdout);
    printFloat(stdout, topSpeed, 1);
    fputs("\n Turn=", stdout);
    printFloat(stdout, turnSpeed, 1);
    fputs("\n Kp=", stdout);
    printFloat(stdout, speedKp, 1);
    fputs("\n Ki=", stdout);
    printFloat(stdout, speedKi, 1);
    fputs("\n Kd=", stdout);
    printFloat(stdout, speedKd, 1);
    fputc('\n', stdout);

    // Print out steer configuration data
    fputs("Steering:", stdout);
    fputs("\n steerZero=", stdout);
    printFloat(stdout, steerZero, 3);
    fputs("\n steerScale=", stdout);
    printFloat(stdout, steerScale, 4);
    fputc('\n', stdout);

    // Print out nav config data
    fputs("Navigation:", stdout);
    fputs("\n Intercept dist=", stdout);
    printFloat(stdout, interceptDist, 1);
    fputs("\n Waypoint dist=", stdout);
    printFloat(stdout, waypointDist, 1);
    fputs("\n Brake dist=", stdout);
    printFloat(stdout, brakeDist, 1);
    fputs("\n Min turn radius=", stdout);
    printFloat(stdout, minRadius, 1);
    fputc('\n', stdout);

    // TODO 3 Print curb configuration
	// curbThreshold
	// curbGain

	// TODO 3 Print gyro/mag configuration

    // Print Waypoint configuration
    fputs("Waypoints:", stdout);
    for (unsigned int w = 0; w < MAXWPT && w < wptCount; w++) {
        fputs("\n waypoint #", stdout);
        printInt(stdout, w);
        fputs(" (", stdout);
        printFloat(stdout, cwpt[w].x, 4);
        fputs(", ", stdout);
        printFloat(stdout, cwpt[w].y, 4);
        fputs(") lat: ", stdout);
        printFloat(stdout, wpt[w].latitude(), 7);
        fputs(" lon: ", stdout);
        printFloat(stdout, wpt[w].longitude(), 7);
        fputs(", topspeed: ", stdout);
        printFloat(stdout, topSpeed + wptTopSpeedAdj[w], 1);
        fputs(", turnspeed: ", stdout);
        printFloat(stdout, turnSpeed + wptTurnSpeedAdj[w], 1);
    }
    fputc('\n', stdout);
}
