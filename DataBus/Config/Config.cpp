#define MAXBUF 64

#include "Config.h"
#include "SDHCFileSystem.h"
#include "GeoPosition.h"
#include "globals.h"
#include "util.h"

#define CONFIGFILE	"/etc/config.txt"

extern Serial pc;

// TODO: 3: mod waypoints to include speed after waypoint

Config::Config():
    loaded(false)
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
    bool declFound = false;
    bool confStatus = false;
    
    // Just to be safe let's wait
    //wait(2.0);

    pc.printf("opening config file...\n");
    
    fp = fopen(CONFIGFILE, "r");
    if (fp == 0) {
        pc.printf("Could not open %s\n", CONFIGFILE);
    } else {
        wptCount = 0;
        declination = 0.0;
        while (!feof(fp)) {
            fgets(buf, MAXBUF-1, fp);
            p = split(tmp, buf, MAXBUF, ',');           // split off the first field
            switch (tmp[0]) {
                case 'B' :
                    p = split(tmp, p, MAXBUF, ',');     // threshold distance for curb avoid
                    curbThreshold = cvstof(tmp);
                    p = split(tmp, p, MAXBUF, ',');     // steering gain for curb avoid
                    curbGain = cvstof(tmp);
                    break;
                case 'W' :                              // Waypoint
                    p = split(tmp, p, MAXBUF, ',');     // split off the latitude to tmp
                    lat = cvstof(tmp);
                    p = split(tmp, p, MAXBUF, ',');     // split off the longitude to tmp
                    lon = cvstof(tmp);
                    if (wptCount < MAXWPT) {
                        wpt[wptCount].set(lat, lon);
                        wptCount++;
                    }
                    break;
                case 'G' :                              // GPS
                    p = split(tmp, p, MAXBUF, ',');
                    gpsBaud = atoi(tmp);                // baud rate for gps
                    p = split(tmp, p, MAXBUF, ',');
                    gpsType = atoi(tmp);
                    break;
                case 'Y' :                              // Gyro Bias
                    p = split(tmp, p, MAXBUF, ',');     // split off the declination to tmp
                    gyroBias = (float) cvstof(tmp);
                    break;
                case 'D' :                              // Compass Declination
                    p = split(tmp, p, MAXBUF, ',');     // split off the declination to tmp
                    declination = (float) cvstof(tmp);
                    declFound = true;
                    break;
                case 'N' :                                  // Navigation and Turning    
                    p = split(tmp, p, MAXBUF, ',');
                    interceptDist = (float) cvstof(tmp);    // intercept distance for steering algorithm
                    p = split(tmp, p, MAXBUF, ',');
                    waypointDist = (float) cvstof(tmp);     // distance before waypoint switch
                    p = split(tmp, p, MAXBUF, ',');
                    brakeDist = (float) cvstof(tmp);        // distance at which braking starts
                    p = split(tmp, p, MAXBUF, ',');
                    minRadius = (float) cvstof(tmp);        // minimum turning radius
                    break;
                case 'M' :                              // Magnetometer
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
                    break;
                case 'R' : // Steering configuration
                    p = split(tmp, p, MAXBUF, ',');
                    steerZero = cvstof(tmp);            // servo center setting
                    p = split(tmp, p, MAXBUF, ',');
                    steerGain = cvstof(tmp);            // steering angle multiplier
                    p = split(tmp, p, MAXBUF, ',');
                    steerGainAngle = cvstof(tmp);       // angle below which steering gain takes effect
                    break;
                case 'S' :                              // Throttle configuration
                    p = split(tmp, p, MAXBUF, ',');
                    escMin = atoi(tmp);                 // minimum esc (brake) setting
                    p = split(tmp, p, MAXBUF, ',');     
                    escZero = atoi(tmp);                // esc center/zero setting
                    p = split(tmp, p, MAXBUF, ',');     
                    escMax = atoi(tmp);                 // maximum esc setting
                    p = split(tmp, p, MAXBUF, ',');   
                    topSpeed = cvstof(tmp);             // desired top speed
                    p = split(tmp, p, MAXBUF, ',');   
                    turnSpeed = cvstof(tmp);            // speed to use within brake distance of waypoint
                    p = split(tmp, p, MAXBUF, ',');
                    startSpeed = cvstof(tmp);            // speed to use at start
                    p = split(tmp, p, MAXBUF, ',');
                    speedKp = cvstof(tmp);              // speed PID: proportional gain
                    p = split(tmp, p, MAXBUF, ',');     
                    speedKi = cvstof(tmp);              // speed PID: integral gain
                    p = split(tmp, p, MAXBUF, ',');     
                    speedKd = cvstof(tmp);              // speed PID: derivative gain
    
                    break;
                case 'E' :
                    p = split(tmp, p, MAXBUF, ',');     
                    compassGain = (float) cvstof(tmp);  // not used (DCM)
                    p = split(tmp, p, MAXBUF, ',');    
                    yawGain = (float) cvstof(tmp);      // not used (DCM)
                    break;
                default :
                    break;
            } // switch
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
