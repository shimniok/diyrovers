#include "mbed.h"
#include "util.h"
#include "globals.h"
#include "updater.h"
#include "Config.h"
#include "Actuators.h"
#include "Sensors.h"
#include "SystemState.h"
#include "Venus638flpx.h"
#include "Steering.h"
#include "Servo.h"
#include "Mapping.h"
#include "CartPosition.h"
#include "GeoPosition.h"
#include "kalman.h"

#define UPDATE_PERIOD 0.010             // update period in s

// TODO 3 put x,y,z defines somewhere else
#define _x_ 0
#define _y_ 1
#define _z_ 2

// TODO 3 parameterize LED update and feed through event queue (or something)
DigitalOut useGpsStat(LED1);

SystemState nowState;

// The below is for main loop at 50ms = 20hz operation
//#define CTRL_SKIP 2 // 100ms, 10hz, control update
//#define MAG_SKIP 1  // 50ms, 20hz, magnetometer update
//#define LOG_SKIP 1  // 50ms, 20hz, log entry entered into fifo

// The following is for main loop at 10ms = 100hz
#define CTRL_SKIP 5     // 50ms control update
#define MAG_SKIP 2      // 20ms magnetometer update
#define LOG_SKIP 2
#define PWR_SKIP 10		// 100ms log entry entered into fifo

Ticker sched;                           // scheduler for interrupt driven routines

int control_count=CTRL_SKIP;			// update control outputs every so often
int update_count=MAG_SKIP;              // call Update_mag() every update_count calls to schedHandler()
int power_count=PWR_SKIP;				// read power sensors every so often
int log_count=LOG_SKIP;                 // buffer a new status entry for logging every log_count calls to schedHandler
int tReal;                              // calculate real elapsed time


// TODO: 3 better encapsulation, please
extern DigitalOut gpsStatus;			// GPS status LED
extern DigitalOut ahrsStatus;           // AHRS status LED
extern Sensors sensors;
extern Mapping mapper;
extern Steering steerCalc;              // steering calculator
extern Timer timer;

// Navigation
extern Config config;
int nextWaypoint = 0;                   // next waypoint destination
int lastWaypoint = 1;
double bearing;                         // bearing to next waypoint
double distance;                        // distance to next waypoint
float steerAngle;                       // steering angle

// Throttle PID
float speedDt=0;                        // dt for the speed PID
float integral=0;                       // error integral for speed PID
float lastError=0;                      // previous error, used for calculating derivative
volatile bool go=false;                 // initiate throttle (or not)
float desiredSpeed;                     // speed set point
float nowSpeed;

// Pose Estimation
bool initNav = true;                    // initialize navigation estimates
bool doLog = false;                     // determines when to start and stop entering log data
float initialHeading=-999;              // initial heading
CartPosition cartHere;                  // position estimate, cartesian
GeoPosition here;                       // position estimate, lat/lon
int timeZero=0;
int lastTime=-1;                        // used to calculate dt for KF
int thisTime;                           // used to calculate dt for KF
float dt;                               // dt for the KF
float estLagHeading = 0;                // lagged heading estimate
float errHeading;                       // error between gyro hdg estimate and gps hdg estimate
float gyroBias=0;                       // exponentially weighted moving average of gyro error
float Ag = (2.0/(1000.0+1.0));          // Gyro bias filter alpha, gyro; # of 10ms steps
float Kbias = 0.995;            
float filtErrRate = 0;
float biasErrAngle = 0;

// Gyro/heading history
#define MAXHIST 128 // must be multiple of 0x08
#define inc(x)  (x) = ((x)+1)&(MAXHIST-1)
struct history_rec {
    float x;        // x coordinate
    float y;        // y coordinate
    float hdg;      // heading
    float dist;     // distance
    float gyro;     // heading rate
    float dt;       // delta time
} history[MAXHIST]; // fifo for sensor data, position, heading, dt

int hCount;         // history counter; we can go back in time to reference gyro history
int now = 0;        // fifo input index, latest entry
int prev = 0;       // previous fifo iput index, next to latest entry
int lag = 0;        // fifo output index, entry from 1 second ago (sensors.gps.lag entries prior)
int lagPrev = 0;    // previous fifo output index, 101 entries prior
int logCounter = 0;


/** attach update to Ticker */
void startUpdater()
{
    // Initialize logging buffer
    // Needs to happen after we've reset the millisecond timer and after
    // the schedHandler() fires off at least once more with the new time
    sched.attach(&update, UPDATE_PERIOD);
}

/** set flag to initialize navigation at next schedHandler() call */
void restartNav()
{
	__disable_irq();
    go = false;
    fifo_reset();
    initNav = true;
    __enable_irq();

    return;
}

/** instruct the controller to start running */
void beginRun()
{
	__disable_irq();
    go = true;
    fifo_reset();
    timeZero = thisTime; // initialize
    __enable_irq();

    return;
}

/** instruct the controller that we're done with the run */
void endRun()
{
    go = false;
    initNav = true;

    return;
}

/** get elasped time in update loop */
int getUpdateTime()
{
    return tReal;
}

/** Set the desired speed for the controller to attain */
void setSpeed(const float speed)
{
    if (desiredSpeed != speed) {
        desiredSpeed = speed;
        integral = 0; // reset the error integral
    }
    return;
}

// TODO 2 split update function into multiple tasks, one for reading, one for estimation, control?

/** update() runs the data collection, estimation, steering control, and throttle control */
void update()
{
    tReal = timer.read_us();
    bool useGps=false;

    ahrsStatus = 0;
    thisTime = timer.read_ms();
    dt = (lastTime < 0) ? 0 : ((float) thisTime - (float) lastTime) / 1000.0; // first pass let dt=0
    lastTime = thisTime;

    // Add up dt to speedDt
    // We're adding up distance over several update() calls so have to keep track of total time
    speedDt += dt;
    
    //////////////////////////////////////////////////////////////////////////////
    // NAVIGATION INIT
    //////////////////////////////////////////////////////////////////////////////
    // initNav is set with call to restartNav() when the "go" button is pressed.  Up
    // to 10ms later this function is called and the code below will initialize the
    // dead reckoning position and estimated position variables
    // 
    if (initNav == true) {
        initNav = false;
        here.set(config.wpt[0]);
        nextWaypoint = 1; // Point to the next waypoint; 0th wpt is the starting point
        lastWaypoint = 0;
        
        // Initialize lag estimates
        //lagHere.set( here );
        hCount = 2; // lag entry and first now entry are two entries
        now = 0;
        // initialize what will become lag data in 1 second from now
        history[now].dt = 0;
        history[now].dist = 0;
        // initial position is waypoint 0
        history[now].x = config.cwpt[0].x;
        history[now].y = config.cwpt[0].y;
        cartHere.set(history[now].x, history[now].y);
        // initialize heading to bearing between waypoint 0 and 1
        //history[now].hdg = here.bearingTo(config.wpt[nextWaypoint]);
        initialHeading = history[now].hdg = cartHere.bearingTo(config.cwpt[nextWaypoint]);
        //history[now].ghdg = history[now].hdg;
        // Initialize Kalman Filter
        headingKalmanInit(initialHeading);
        // point next fifo input to slot 1, slot 0 occupied/initialized, now
        lag = 0;
        lagPrev = 0;
        prev = now; // point to the most recently entered data
        now = 1;    // new input slot
    }

    //////////////////////////////////////////////////////////////////////////////
    // SENSOR UPDATES
    //////////////////////////////////////////////////////////////////////////////
    if (power_count-- <= 0) {
    	sensors.Read_Power();
    	power_count = PWR_SKIP;
    }
    sensors.Read_Encoders(); 
    sensors.Read_Gyro(); 
    sensors.Read_Rangers();
    sensors.Read_Accel();
    //sensors.Read_Camera();


    //////////////////////////////////////////////////////////////////////////////
    // Obtain GPS data                        
    //////////////////////////////////////////////////////////////////////////////

    // synchronize when RMC and GGA sentences received w/ AHRS
    // Do this in schedHandler??  GPS data is coming in not entirely in sync
    // with the logging info
    if (sensors.gps.available()) {
        // update system status struct for logging
        gpsStatus = !gpsStatus;
        nowState.gpsLatitude = sensors.gps.latitude();
        nowState.gpsLongitude = sensors.gps.longitude();
        nowState.gpsHDOP = sensors.gps.hdop();
        nowState.gpsCourse_deg = sensors.gps.heading_deg();
        nowState.gpsSpeed_mps = sensors.gps.speed_mps(); // if need to convert from mph to mps, use *0.44704
        nowState.gpsSats = sensors.gps.sat_count();
        
        // May 26, 2013, moved the useGps setting in here, so that we'd only use the GPS heading in the
        // Kalman filter when it has just been received. Before this I had a bug where it was using the
        // last known GPS data at every call to this function, meaning the more stale the GPS data, the more
        // it would likely throw off the GPS/gyro error term. Hopefully this will be a tad more acccurate.
        // Only an issue when heading is changing, I think.
        
        // GPS heading is unavailable from this particular GPS at speeds < 0.5 mph
        // Also, best to only use GPS if we've got at least 4 sats active -- really should be like 5 or 6
        // Finally, it takes 3-5 secs of runtime for the gps heading to converge.
        useGps = (nowState.gpsSats > 4 &&
                  nowState.lrEncSpeed > 1.0 &&
                  nowState.rrEncSpeed > 1.0 &&
                  (thisTime-timeZero) > 3000); // gps hdg converges by 3-5 sec.                
    }

    useGpsStat = useGps;
    
    //////////////////////////////////////////////////////////////////////////////
    // HEADING AND POSITION UPDATE
    //////////////////////////////////////////////////////////////////////////////

    // TODO: 3 Position filtering
    //    position will be updated based on heading error from heading estimate

    // So the big pain in the ass is that the GPS data coming in represents the
    // state of the system N seconds ago. Up to a full second of lag despite running
    // at 10hz (or whatever).  So if we try and fuse a lagged gps heading with a
    // relatively current gyro heading rate, the gyro is ignored and the heading
    // estimate lags reality
    //
    // In real life testing, the robot steering control was highly unstable with
    // too much gain (typical of any negative feedback system with a very high
    // phase shift and too much feedback). It'd drive around in circles trying to
    // hunt for the next waypoint.  Dropping the gain restored stability but the
    // steering overshot significantly, because of the lag.  It sort of worked but
    // it was ugly. So here's my lame attempt to fix all this. 
    // 
    // We'll find out how much error there is between gps heading and the integrated
    // gyro heading from a second ago.

    // Save current data into history fifo to use 1 second from now
    history[now].dist = (sensors.lrEncDistance + sensors.rrEncDistance) / 2.0; // current distance traveled
    history[now].gyro = sensors.gyro[_z_];  // current raw gyro data
    history[now].dt = dt; // current dt, to address jitter
    history[now].hdg = clamp360( history[prev].hdg + dt*(history[now].gyro - gyroBias) ); // compute current heading from current gyro
    float r = PI/180 * history[now].hdg;
    history[now].x = history[prev].x + history[now].dist * sin(r);    // update current position
    history[now].y = history[prev].y + history[now].dist * cos(r);

    // We can't do anything until the history buffer is full
    if (hCount < sensors.gps.lag) {
        estLagHeading = history[now].hdg;
        // Until the fifo is full, only keep track of current gyro heading
        hCount++; // after n iterations the fifo will be full
    } else {
        // Now that the buffer is full, we'll maintain a Kalman Filter estimate that is
        // time-shifted to the past because the GPS output represents the system state from
        // the past. We'll also correct our history of gyro readings from the past to the
        // present
        
        ////////////////////////////////////////////////////////////////////////////////
        // UPDATE LAGGED ESTIMATE
        
        // Recover data from 1 second ago which will be used to generate
        // updated lag estimates

        // This represents the best estimate for heading... for one second ago
        // If we do nothing else, the robot will think it is located at a position 1 second behind
        // where it actually is. That means that any control feedback needs to have a longer time
        // constant than 1 sec or the robot will have unstable heading correction.
        // Use gps data when available, always use gyro data

        // Clamp heading to initial heading when we're not moving; hopefully this will
        // give the KF a head start figuring out how to deal with the gyro
        //
        // TODO 1 maybe we should only call the gps version after moving
        if (go) {
            estLagHeading = headingKalman(history[lag].dt, nowState.gpsCourse_deg, useGps, history[lag].gyro, true);
        } else {    
            estLagHeading = headingKalman(history[lag].dt, initialHeading, true, history[lag].gyro, true);
        }

        // TODO 1 are we adding history lag to lagprev or should we add lag+1 to lag or what?
        // Update the lagged position estimate
        history[lag].x = history[lagPrev].x + history[lag].dist * sin(estLagHeading);
        history[lag].y = history[lagPrev].y + history[lag].dist * cos(estLagHeading);
        
        ////////////////////////////////////////////////////////////////////////////////
        // UPDATE CURRENT ESTIMATE

        // Now we need to re-calculate the current heading and position, starting with the most recent
        // heading estimate representing the heading 1 second ago.
        //        
        // Nuance: lag and now have already been incremented so that works out beautifully for this loop
        //
        // Heading is easy. Original heading - estimated heading gives a tiny error. Add this to all the historical
        // heading data up to present.
        //
        // For position re-calculation, we iterate sensors.gps.lag times up to present record. Haversine is way, way too slow,
        // trig calcs is marginal. Update rate is 10ms and we can't hog more than maybe 2-3ms as the outer
        // loop has logging work to do. Rotating each point is faster; pre-calculate sin/cos, etc. for rotation
        // matrix.

        // Low pass filter the error correction.  Multiplying by 0.01, it takes sensors.gps.lag updates to correct a
        // consistent error; that's 0.10s/0.01 = 1 sec.  0.005 is 2 sec, 0.0025 is 4 sec, etc.
        errHeading = clamp180(estLagHeading - history[lag].hdg) * 0.01;  // lopass filter error angle

        //fprintf(stdout, "%d %.2f, %.2f, %.4f %.4f\n", lag, estLagHeading, history[lag].hdg, estLagHeading - history[lag].hdg, errHeading);
        float cosA = cos(errHeading * PI / 180.0);
        float sinA = sin(errHeading * PI / 180.0);
        // Update position & heading from history[lag] through history[now]
        int i = lag;
        for (int j=0; j < sensors.gps.lag; j++) {
            //fprintf(stdout, "i=%d n=%d l=%d\n", i, now, lag);
            // Correct gyro-calculated headings from past to present including history[lag].hdg
            history[i].hdg = clamp360(history[i].hdg + errHeading);
            //if (history[i].hdg >= 360.0) history[i].hdg -= 360.0;
            //if (history[i].hdg < 0) history[i].hdg += 360.0;
            // Rotate x, y by errHeading around pivot point; no need to rotate pivot point (j=0)
            if (j > 0) {
                float dx = history[i].x-history[lag].x;
                float dy = history[i].y-history[lag].y;
                // rotation matrix
                history[i].x = history[lag].x + dx*cosA - dy*sinA;
                history[i].y = history[lag].y + dx*sinA + dy*cosA;
            }
            inc(i);
        }
        // increment lag pointer and wrap        
        lagPrev = lag;
        inc(lag);
    }
    // "here" is the current position
    cartHere.set(history[now].x, history[now].y);

    //////////////////////////////////////////////////////////////////////////////
    // NAVIGATION UPDATE
    //////////////////////////////////////////////////////////////////////////////
    
    bearing = cartHere.bearingTo(config.cwpt[nextWaypoint]);
    distance = cartHere.distanceTo(config.cwpt[nextWaypoint]);
    float prevDistance = cartHere.distanceTo(config.cwpt[lastWaypoint]);

    // if within config.waypointDist distance threshold move to next waypoint
    // TODO 3 figure out how to output feedback on wpt arrival external to this function
    if (go) {

        // if we're within brakeDist of next or previous waypoint, run @ turn speed
        // This would normally mean we run at turn speed until we're brakeDist away
        // from waypoint 0, but we trick the algorithm by initializing prevWaypoint to waypoint 1
        if ( (thisTime - timeZero) < 3000 ) {
            setSpeed( config.startSpeed );
        } else if (distance < config.brakeDist || prevDistance < config.brakeDist) {
            setSpeed( config.turnSpeed );
            // TODO 2 setSpeed( config.wptTurnSpeed[nextWaypoint] );
        } else {
            setSpeed( config.topSpeed );
            // TODO 2 setSpeed( config.wptTopSpeed[nextWaypoint] );
        }

        if (distance < config.waypointDist) {
            //fprintf(stdout, "Arrived at wpt %d\n", nextWaypoint);
            //speaker.beep(3000.0, 0.5); // non-blocking
            lastWaypoint = nextWaypoint;
            nextWaypoint++;
        }
        
    } else {
        setSpeed( 0.0 ); // TODO if we set speed 0 here but in control area it sets it based on desired speed now what?
    }
    // Are we at the last waypoint?
    // currently handled external to this routine
        
    //////////////////////////////////////////////////////////////////////////////
    // OBSTACLE DETECTION & AVOIDANCE
    //////////////////////////////////////////////////////////////////////////////
    // TODO 3 obstacle detection, vision based detection, etc.; avoidance


    //////////////////////////////////////////////////////////////////////////////
    // CONTROL UPDATE
    //////////////////////////////////////////////////////////////////////////////

    if (go) // TODO 1 temporarily only enable control output when in go mode
    if (--control_count == 0) {
  
        steerAngle = steerCalc.pathPursuitSA(history[now].hdg, history[now].x, history[now].y,
                                             config.cwpt[lastWaypoint].x, config.cwpt[lastWaypoint].y,
                                             config.cwpt[nextWaypoint].x, config.cwpt[nextWaypoint].y);
        
        // Apply gain factor for near straight line
        // TODO 3 figure out a better, continuous way to deal with steering gain and put it in steering abstraction!
        //if (fabs(steerAngle) < config.steerGainAngle) steerAngle *= config.steerGain;

        // Curb avoidance
        /*
        if (sensors.rightRanger < config.curbThreshold) {
            steerAngle -= config.curbGain * (config.curbThreshold - sensors.rightRanger);
        }
        */

        setSteering( steerAngle );

        // PID control for throttle
        // TODO: 3 move all this PID crap into Actuators.cpp
        // TODO: 3 probably should do KF or something for speed/dist; need to address GPS lag, too
        // if nothing else, at least average the encoder speed over mult. samples
        if (desiredSpeed <= 0.1) {
            setThrottle( config.escZero );
        } else {
            // TODO 3 filter speed
        	// e.g., nowSpeed = 0.8*nowSpeed + 0.2*sensors.encSpeed; and/or median filter and/or logic checks
        	// may also want to filter against GPS speed? would involve lag compensation though, thus memory expensive
            nowSpeed = sensors.encSpeed;
        	// PID loop for throttle control
            // http://www.codeproject.com/Articles/36459/PID-process-control-a-Cruise-Control-example
            float error = desiredSpeed - nowSpeed; 
            // track error over time, scaled to the timer interval
            integral += (error * speedDt);
            // determine the amount of change from the last time checked
            float derivative = (error - lastError) / speedDt; 
            // calculate how much to drive the output in order to get to the 
            // desired setpoint. 
            float output = config.escZero + (config.speedKp * error) + (config.speedKi * integral) + (config.speedKd * derivative);
            //fprintf(stdout, "s=%.1f d=%.1f o=%d\n", nowSpeed, desiredSpeed, output);
            setThrottle( output );
            // remember the error for the next time around.
            lastError = error; 
        }

        speedDt = 0; // reset dt to begin counting for next time
        control_count = CTRL_SKIP;
    }      

    //////////////////////////////////////////////////////////////////////////////
    // DATA FOR LOGGING
    //////////////////////////////////////////////////////////////////////////////

    // Log Data Timestamp    
    //nowState.millis = thisTime;
    nowState.millis = thisTime;
    nowState.voltage = sensors.voltage;
    nowState.current = sensors.current;
    for (int i=0; i < 3; i++) {
        nowState.m[i] = sensors.m[i];
        nowState.g[i] = sensors.g[i];
        nowState.gyro[i] = sensors.gyro[i];
        nowState.a[i] = sensors.a[i];
    }
    nowState.gTemp = sensors.gTemp;
    nowState.lrEncSpeed = sensors.lrEncSpeed;
    nowState.rrEncSpeed = sensors.rrEncSpeed;
    nowState.lrEncDistance += sensors.lrEncDistance;
    nowState.rrEncDistance += sensors.rrEncDistance;
    //state.encHeading += (state.lrEncDistance - state.rrEncDistance) / TRACK;
    nowState.estHeading = history[now].hdg; // gyro heading estimate, current, corrected
    nowState.estLagHeading = history[lag].hdg; // gyro heading, estimate, lagged
    mapper.cartToGeo(cartHere, &here);
    nowState.estLatitude = here.latitude();
    nowState.estLongitude = here.longitude();
    nowState.estX = history[now].x;
    nowState.estY = history[now].y;
    nowState.bearing = bearing;
    nowState.distance = distance;
    nowState.nextWaypoint = nextWaypoint;
    nowState.gbias = gyroBias;
    nowState.errHeading = errHeading;
    //state.leftRanger = sensors.leftRanger;
    //state.rightRanger = sensors.rightRanger;
    //state.centerRanger = sensors.centerRanger;
    nowState.steerAngle = steerAngle;
    // Copy AHRS data into logging data
    //state.roll = ToDeg(ahrs.roll);
    //state.pitch = ToDeg(ahrs.pitch);
    //state.yaw = ToDeg(ahrs.yaw);

    // Periodically, we enter a new SystemState into the FIFO buffer
    // The main loop handles logging and will catch up to us provided
    // we feed in new log entries slowly enough.
    //if (go) {
    	if (--log_count == 0) {
			fifo_push(&nowState);
			state_clear(&nowState);
			log_count = LOG_SKIP;       // reset counter
    	}
    //}

    // increment history fifo pointers with wrap-around
    prev = now;
    inc(now);

    // timing
    tReal = timer.read_us() - tReal;

    ahrsStatus = 1;
}
