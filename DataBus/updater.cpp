#include "mbed.h"
#include "util.h"
#include "globals.h"
#include "updater.h"
#include "Config.h"
#include "Actuators.h"
#include "Sensors.h"
#include "SystemState.h"
#include "Venus638flpx.h"
//#include "Ublox6.h"
#include "Steering.h"
#include "Servo.h"
#include "Mapping.h"
#include "CartPosition.h"
#include "GeoPosition.h"
#include "kalman.h"

#define UPDATE_PERIOD 0.010             // update period in s

#define _x_ 0
#define _y_ 1
#define _z_ 2

// The below is for main loop at 50ms = 20hz operation
//#define CTRL_SKIP 2 // 100ms, 10hz, control update
//#define MAG_SKIP 1  // 50ms, 20hz, magnetometer update
//#define LOG_SKIP 1  // 50ms, 20hz, log entry entered into fifo

// The following is for main loop at 10ms = 100hz
#define HDG_LAG 40      // Number of update steps that the GPS report is behind realtime
#define CTRL_SKIP 5     // 50ms (20hz), control update
#define MAG_SKIP 2      // 20ms (50hz), magnetometer update
#define LOG_SKIP 2      // 20ms (50hz), log entry entered into fifo
//#define LOG_SKIP 5    // 50ms (20hz), log entry entered into fifo

Ticker sched;                           // scheduler for interrupt driven routines

int control_count=CTRL_SKIP;
int update_count=MAG_SKIP;              // call Update_mag() every update_count calls to schedHandler()
int log_count=0;                        // buffer a new status entry for logging every log_count calls to schedHandler
int tReal;                              // calculate real elapsed time
int bufCount=0;

extern DigitalOut gpsStatus;

// TODO: 3 better encapsulation, please
extern Sensors sensors;
extern SystemState state[SSBUF];
extern unsigned char inState;
extern unsigned char outState;
extern bool ssBufOverrun;
extern Mapping mapper;
extern Steering steerCalc;              // steering calculator
extern Timer timer;
extern DigitalOut ahrsStatus;           // AHRS status LED

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
bool go=false;                          // initiate throttle (or not)
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
float estLagHeading = 0;                   // lagged heading estimate
//GeoPosition lagHere;                  // lagged position estimate; use here as the current position estimate
float errHeading;                         // error between gyro hdg estimate and gps hdg estimate
float gyroBias=0;                       // exponentially weighted moving average of gyro error
float Ag = (2.0/(1000.0+1.0));          // Gyro bias filter alpha, gyro; # of 10ms steps
float Kbias = 0.995;            
float filtErrRate = 0;
float biasErrAngle = 0;

#define MAXHIST 128 // must be multiple of 0x08
#define inc(x)  (x) = ((x)+1)&(MAXHIST-1)
struct history_rec {
    float x;        // x coordinate
    float y;        // y coordinate
    float hdg;      // heading
    float dist;     // distance
    float gyro;     // heading rate
    //float ghdg;   // uncorrected gyro heading
    float dt;       // delta time
} history[MAXHIST]; // fifo for sensor data, position, heading, dt

int hCount;         // history counter; one > HDG_LAG, we can go back in time to reference gyro history
int now = 0;        // fifo input index, latest entry
int prev = 0;       // previous fifo iput index, next to latest entry
int lag = 0;        // fifo output index, entry from 1 second ago (HDG_LAG entries prior)
int lagPrev = 0;    // previous fifo output index, 101 entries prior

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
    go = false;
    inState = outState = 0;    
    initNav = true;
    return;
}

/** instruct the controller to start running */
void beginRun()
{
    go = true;
    inState = outState = 0;    
    timeZero = thisTime; // initialize 
    bufCount = 0;
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
void setSpeed(float speed) 
{
    if (desiredSpeed != speed) {
        desiredSpeed = speed;
        integral = 0; // reset the error integral
    }
    return;
}

/** update() runs the data collection, estimation, steering control, and throttle control */
void update()
{
    tReal = timer.read_us();
    bool useGps=false;

    // TODO 1 do we need to set up the dt stuff after initialization?

    ahrsStatus = 0;
    thisTime = timer.read_ms();
    dt = (lastTime < 0) ? 0 : ((float) thisTime - (float) lastTime) / 1000.0; // first pass let dt=0
    lastTime = thisTime;

    // Add up dt to speedDt
    // We're adding up distance over several update() calls so have to keep track of total time
    speedDt += dt;
    
    // Log Data Timestamp    
    int timestamp = timer.read_ms();

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
        
        // TODO 1 need to start off, briefly, at a slow speed
        
        // Initialize lag estimates
        //lagHere.set( here );
        hCount = 2; // lag entry and first now entry are two entries
        now = 0; 
        // initialize what will become lag data in 1 second from now
        history[now].dt = 0;
        history[now].dist = 0;
        // initial position is waypoint 0
        history[now].x = config.cwpt[0]._x;
        history[now].y = config.cwpt[0]._y;
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

    // TODO 3 This really should run infrequently
    sensors.Read_Power();

    sensors.Read_Encoders(); 
    // really need to do some filtering on the speed
    //nowSpeed = 0.8*nowSpeed + 0.2*sensors.encSpeed; 
    nowSpeed = sensors.encSpeed;

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
        state[inState].gpsLatitude = sensors.gps.latitude();
        state[inState].gpsLongitude = sensors.gps.longitude();
        state[inState].gpsHDOP = sensors.gps.hdop();
        state[inState].gpsCourse_deg = sensors.gps.heading_deg();
        state[inState].gpsSpeed_mps = sensors.gps.speed_mps(); // if need to convert from mph to mps, use *0.44704
        state[inState].gpsSats = sensors.gps.sat_count();
        
        // May 26, 2013, moved the useGps setting in here, so that we'd only use the GPS heading in the
        // Kalman filter when it has just been received. Before this I had a bug where it was using the
        // last known GPS data at every call to this function, meaning the more stale the GPS data, the more
        // it would likely throw off the GPS/gyro error term. Hopefully this will be a tad more acccurate.
        // Only an issue when heading is changing, I think.
        
        // GPS heading is unavailable from this particular GPS at speeds < 0.5 mph
        // Also, best to only use GPS if we've got at least 4 sats active -- really should be like 5 or 6
        // Finally, it takes 3-5 secs of runtime for the gps heading to converge.
        useGps = (state[inState].gpsSats > 4 &&
                  state[inState].lrEncSpeed > 1.0 &&
                  state[inState].rrEncSpeed > 1.0 &&
                  (thisTime-timeZero) > 3000); // gps hdg converges by 3-5 sec.                
    }

    DigitalOut useGpsStat(LED1);
    useGpsStat = useGps;
    
    //////////////////////////////////////////////////////////////////////////////
    // HEADING AND POSITION UPDATE
    //////////////////////////////////////////////////////////////////////////////

    // TODO: 2 Position filtering
    //    position will be updated based on heading error from heading estimate
    // TODO: 2 Distance/speed filtering
    //    this might be useful, but not sure it's worth the effort

    // So the big pain in the ass is that the GPS data coming in represents the
    // state of the system ~1s ago. Yes, a full second of lag despite running
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
    if (hCount < HDG_LAG) {
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
            estLagHeading = headingKalman(history[lag].dt, state[inState].gpsCourse_deg, useGps, history[lag].gyro, true);
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
        // For position re-calculation, we iterate HDG_LAG times up to present record. Haversine is way, way too slow,
        // trig calcs is marginal. Update rate is 10ms and we can't hog more than maybe 2-3ms as the outer
        // loop has logging work to do. Rotating each point is faster; pre-calculate sin/cos, etc. for rotation
        // matrix.

        // Low pass filter the error correction.  Multiplying by 0.01, it takes HDG_LAG updates to correct a 
        // consistent error; that's 0.10s/0.01 = 1 sec.  0.005 is 2 sec, 0.0025 is 4 sec, etc.
        errHeading = clamp180(estLagHeading - history[lag].hdg) * 0.01;  // lopass filter error angle

        //fprintf(stdout, "%d %.2f, %.2f, %.4f %.4f\n", lag, estLagHeading, history[lag].hdg, estLagHeading - history[lag].hdg, errHeading);
        float cosA = cos(errHeading * PI / 180.0);
        float sinA = sin(errHeading * PI / 180.0);
        // Update position & heading from history[lag] through history[now]
        int i = lag;
        // TODO 2 parameterize heading lag -- for uBlox it seems to be ~ 600ms, for Venus, about 1000ms
        for (int j=0; j < HDG_LAG; j++) {
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
        } else {
            setSpeed( config.topSpeed );
        }

        if (distance < config.waypointDist) {
            //fprintf(stdout, "Arrived at wpt %d\n", nextWaypoint);
            //speaker.beep(3000.0, 0.5); // non-blocking
            lastWaypoint = nextWaypoint;
            nextWaypoint++;
        }
        
    } else {
        setSpeed( 0.0 );
    }
    // Are we at the last waypoint?
    // currently handled external to this routine
        
    //////////////////////////////////////////////////////////////////////////////
    // OBSTACLE DETECTION & AVOIDANCE
    //////////////////////////////////////////////////////////////////////////////
    // TODO 2 limit steering angle based on object detection ?
    // or limit relative brg perhaps?
    // TODO 2 add vision obstacle detection and filtering
    // TODO 2 add ranger obstacle detection and filtering/fusion with vision


    //////////////////////////////////////////////////////////////////////////////
    // CONTROL UPDATE
    //////////////////////////////////////////////////////////////////////////////

    if (--control_count == 0) {
  
        steerAngle = steerCalc.pathPursuitSA(history[now].hdg, history[now].x, history[now].y,
                                             config.cwpt[lastWaypoint]._x, config.cwpt[lastWaypoint]._y,
                                             config.cwpt[nextWaypoint]._x, config.cwpt[nextWaypoint]._y);
        
        // Apply gain factor for near straight line
        // TODO 3 figure out a better, continuous way to deal with steering gain
        if (fabs(steerAngle) < config.steerGainAngle) steerAngle *= config.steerGain;

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
            // PID loop for throttle control
            // http://www.codeproject.com/Articles/36459/PID-process-control-a-Cruise-Control-example
            float error = desiredSpeed - nowSpeed; 
            // track error over time, scaled to the timer interval
            integral += (error * speedDt);
            // determine the amount of change from the last time checked
            float derivative = (error - lastError) / speedDt; 
            // calculate how much to drive the output in order to get to the 
            // desired setpoint. 
            int output = config.escZero + (config.speedKp * error) + (config.speedKi * integral) + (config.speedKd * derivative);
            if (output > config.escMax) output = config.escMax;
            if (output < config.escMin) output = config.escMin;
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

    // Periodically, we enter a new SystemState into the FIFO buffer
    // The main loop handles logging and will catch up to us provided 
    // we feed in new log entries slowly enough.
    // log_count initialized to 0 to begin with forcing initialization below
    // but no further updates until in go mode
    if (go && (log_count > 0)) --log_count;
    // Only enter new SystemState data when in "go" mode (armed to run, run,
    // and end run)
    // We should really only be adding to the state fifo if we're in 'go' mode
    if (log_count <= 0) {
        // Copy data into system state for logging
        inState++;                      // Get next state struct in the buffer
        inState &= SSBUF;               // Wrap around
        ssBufOverrun = (inState == outState);
        // Need to clear out encoder distance counters; these are incremented
        // each time this routine is called.
        // TODO 1 is there anything we can't clear out?
        // TODO 1 use clearState()
        state[inState].lrEncDistance = 0;
        state[inState].rrEncDistance = 0;
        // initialize gps data
        state[inState].gpsLatitude = 0;
        state[inState].gpsLongitude = 0;
        state[inState].gpsHDOP = 0;
        state[inState].gpsCourse_deg = 0;
        state[inState].gpsSpeed_mps = 0;
        state[inState].gpsSats = 0;
        log_count = LOG_SKIP;       // reset counter
        bufCount++;
    }

    // Log Data Timestamp    
    state[inState].millis = timestamp;

    // TODO: 3 recursive filtering on each of the state values
    state[inState].voltage = sensors.voltage;
    state[inState].current = sensors.current;
    for (int i=0; i < 3; i++) {
        state[inState].m[i] = sensors.m[i];
        state[inState].g[i] = sensors.g[i];
        state[inState].gyro[i] = sensors.gyro[i];
        state[inState].a[i] = sensors.a[i];
    }
    state[inState].gTemp = sensors.gTemp;
    state[inState].lrEncSpeed = sensors.lrEncSpeed;
    state[inState].rrEncSpeed = sensors.rrEncSpeed;
    state[inState].lrEncDistance += sensors.lrEncDistance;
    state[inState].rrEncDistance += sensors.rrEncDistance;
    //state[inState].encHeading += (state[inState].lrEncDistance - state[inState].rrEncDistance) / TRACK;
    state[inState].estHeading = history[now].hdg; // gyro heading estimate, current, corrected
    state[inState].estLagHeading = history[lag].hdg; // gyro heading, estimate, lagged
    mapper.cartToGeo(cartHere, &here);
    state[inState].estLatitude = here.latitude();
    state[inState].estLongitude = here.longitude();
    state[inState].estX = history[now].x;
    state[inState].estY = history[now].y;
    state[inState].bearing = bearing;
    state[inState].distance = distance;
    state[inState].nextWaypoint = nextWaypoint;
    state[inState].gbias = gyroBias;
    state[inState].errHeading = errHeading;
    //state[inState].leftRanger = sensors.leftRanger;
    //state[inState].rightRanger = sensors.rightRanger;
    //state[inState].centerRanger = sensors.centerRanger;
    state[inState].steerAngle = steerAngle;
    // Copy AHRS data into logging data
    //state[inState].roll = ToDeg(ahrs.roll);
    //state[inState].pitch = ToDeg(ahrs.pitch);
    //state[inState].yaw = ToDeg(ahrs.yaw);

    // increment fifo pointers with wrap-around
    prev = now;
    inc(now);

    // timing
    tReal = timer.read_us() - tReal;

    ahrsStatus = 1;
}