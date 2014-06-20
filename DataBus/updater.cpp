#include "mbed.h"
#include "devices.h"
#include "util.h"
#include "globals.h"
#include "updater.h"
#include "Config.h"
#include "Sensors.h"
#include "SystemState.h"
#include "Steering.h"
#include "Servo2.h"
#include "Servo.h"
#include "Mapping.h"
#include "CartPosition.h"
#include "GeoPosition.h"
#include "kalman.h"

#define PURE_PURSUIT 1

#define UPDATE_PERIOD 0.010             // update period in s
// TODO 3 put x,y,z defines somewhere else
#define _x_ 0
#define _y_ 1
#define _z_ 2

// TODO 3 parameterize LED update and feed through event queue (or something)
DigitalOut useGpsStat(LED1);

SystemState nowState;
SystemState prevState;

// The below is for main loop at 50ms = 20hz operation
//#define CTRL_SKIP 2 // 100ms, 10hz, control update
//#define MAG_SKIP 1  // 50ms, 20hz, magnetometer update
//#define LOG_SKIP 1  // 50ms, 20hz, log entry entered into fifo

// The following is for main loop at 10ms = 100hz
#define LED_SKIP 20
#define CTRL_SKIP 5     // 50ms control update
#define MAG_SKIP 2      // 20ms magnetometer update
#define LOG_SKIP 2
#define PWR_SKIP 10		// 100ms log entry entered into fifo
Ticker sched;                         // scheduler for interrupt driven routines

int led_count = LED_SKIP;					// blink led slower as heartbeat
int control_count = CTRL_SKIP;			// update control outputs every so often
int update_count = MAG_SKIP; // call Update_mag() every update_count calls to schedHandler()
int power_count = PWR_SKIP;				// read power sensors every so often
int log_count = LOG_SKIP; // buffer a new status entry for logging every log_count calls to schedHandler
int tReal;                              // calculate real elapsed time

extern DigitalOut gpsStatus;

// TODO: 3 better encapsulation, please
extern Sensors sensors;
extern Mapping mapper;
extern Timer timer;
extern DigitalOut updaterStatus;        // AHRS status LED

// Navigation
Steering steering(STEERING);			// Steering actuator
extern Config config;
int nextWaypoint = 0;                   // next waypoint destination
int lastWaypoint = 1;
double heading;						    // current heading
double bearing;                         // bearing to next waypoint
double distance;                        // distance to next waypoint
float steerAngle;                       // steering angle

// Throttle PID
Servo2 esc(THROTTLE);
float speedDt = 0;                        // dt for the speed PID
float integral = 0;                       // error integral for speed PID
float lastError = 0;          // previous error, used for calculating derivative
float desiredSpeed;                     // speed set point
float nowSpeed;

// Flags
volatile bool go = false;                 // initiate throttle (or not)
volatile bool initNav = true;           // initialize navigation estimates

// Pose Estimation
bool doLog = false;       // determines when to start and stop entering log data
float initialHeading = -999;              // initial heading
CartPosition here;                  	// position estimate, cartesian
GeoPosition hereGeo;                    // position estimate, lat/lon
int timeZero = 0;
int lastTime = -1;                        // used to calculate dt for KF
int thisTime;                           // used to calculate dt for KF
float dt;                               // dt for the KF
float estLagHeading = 0;                // lagged heading estimate
float errHeading;        // error between gyro hdg estimate and gps hdg estimate
float Ag = (2.0 / (1000.0 + 1.0)); // Gyro bias filter alpha, gyro; # of 10ms steps
float Kbias = 0.995;
float filtErrRate = 0;
float biasErrAngle = 0;

#define MAXHIST 128 // must be multiple of 0x08
#define inc(x)  (x) = ((x)+1)&(MAXHIST-1)
//struct history_rec {
//	float x;        // x coordinate
//	float y;        // y coordinate
//	float hdg;      // heading
//	float dist;     // distance
//	float gyro;     // heading rate
//	//float ghdg;   // uncorrected gyro heading
//	float dt;       // delta time
//} history[MAXHIST] __attribute__ ((section("AHBSRAM0"))); // fifo for sensor data, position, heading, dt

//static int hCount = 0;// history counter; we can go back in time to reference gyro history
//static int now = 0;        	// fifo input index, latest entry
//static int prev = 0;       	// previous fifo iput index, next to latest entry
//static int lag = 0; // fifo output index, entry from 1 second ago (sensors.gps.lag entries prior)
//static int lagPrev = 0;   	// previous fifo output index, 101 entries prior

void initThrottle() {
	esc = config.escMin;
}

/** attach update to Ticker */
void startUpdater() {
	// Initialize logging buffer
	// Needs to happen after we've reset the millisecond timer and after
	// the schedHandler() fires off at least once more with the new time
	sched.attach(&update, UPDATE_PERIOD);
}

void stopUpdater() {
	sched.detach();
}

/** set flag to initialize navigation at next schedHandler() call */
void restartNav() {
	__disable_irq();
	go = false;
	fifo_reset();
	initNav = true;
	__enable_irq();

	return;
}

/** instruct the controller to start running */
void beginRun() {
	__disable_irq();
	go = true;
	fifo_reset();
	timeZero = thisTime; // initialize
	__enable_irq();

	return;
}

/** instruct the controller that we're done with the run */
void endRun() {
	go = false;
	initNav = true;

	return;
}

/** get elasped time in update loop */
int getUpdateTime() {
	return tReal;
}

/** Set the desired speed for the controller to attain */
void setSpeed(const float speed) {
	if (desiredSpeed != speed) {
		desiredSpeed = speed;
		integral = 0; // reset the error integral
	}
	return;
}

/** Set the steering angle */
void setSteering(const float steerAngle) {
	steering = steerAngle;
}

// TODO 2 put update sections into separate modules, possibly call using timers with varying priorities?

/** update() runs the data collection, estimation, steering control, and throttle control */
void update() {
	tReal = timer.read_us();
	bool useGps = false;

	if (led_count-- <= 0) {
		updaterStatus = !updaterStatus;
		led_count = LED_SKIP;
	}

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

		// initial position is waypoint 0
		hereGeo.set(config.wpt[0]);
		here.set(config.cwpt[0]);

		// Point to the next waypoint; 0th wpt is the starting point
		nextWaypoint = 1;
		lastWaypoint = 0;

		// initialize heading to bearing between waypoint 0 and 1
		heading = nowState.estHeading = here.bearingTo(config.cwpt[nextWaypoint]);
		// point next fifo input to slot 1, slot 0 occupied/initialized, now

		state_clear(&nowState);
	}

#if 0
	//////////////////////////////////////////////////////////////////////////////
	// SENSOR UPDATES
	//////////////////////////////////////////////////////////////////////////////
	if (power_count-- <= 0) {
		sensors.Read_Power();
		power_count = PWR_SKIP;
	}
	sensors.Read_Encoders();
	// TODO 3 really need to do some filtering on the speed
	//   nowSpeed = 0.8*nowSpeed + 0.2*sensors.encSpeed;
	nowSpeed = sensors.encSpeed;

	sensors.Read_Gyro();
	//sensors.Read_Rangers();
	//sensors.Read_Accel();
	//sensors.Read_Camera();
#endif

	//////////////////////////////////////////////////////////////////////////////
	// Obtain GPS data
	//////////////////////////////////////////////////////////////////////////////

	// synchronize when RMC and GGA sentences received w/ AHRS
	// Do this in schedHandler??  GPS data is coming in not entirely in sync
	// with the logging info
	if (sensors.gps.available()) {
		// update system status struct for logging
		gpsStatus = !gpsStatus;

		hereGeo.set(sensors.gps.latitude(), sensors.gps.longitude());
		// Set current Cartesian position to last reported GPS position estimate
		mapper.geoToCart(hereGeo, &here);

		nowState.gpsHDOP = sensors.gps.hdop();
		nowState.gpsSpeed_mps = sensors.gps.speed_mps(); // if need to convert from mph to mps, use *0.44704

		if (nowState.gpsSpeed_mps > config.gpsValidSpeed) {
			// Set current heading to reported GPS heading
			heading = nowState.gpsCourse_deg = sensors.gps.heading_deg();
		}
		nowState.gpsSats = sensors.gps.sat_count();

		// Set current speed to GPS speed
		nowSpeed = nowState.gpsSpeed_mps;
#if 0
		// May 26, 2013, moved the useGps setting in here, so that we'd only use the GPS heading in the
		// Kalman filter when it has just been received. Before this I had a bug where it was using the
		// last known GPS data at every call to this function, meaning the more stale the GPS data, the more
		// it would likely throw off the GPS/gyro error term. Hopefully this will be a tad more acccurate.
		// Only an issue when heading is changing, I think.

		// GPS heading is unavailable from this particular GPS at speeds < 0.5 mph
		// Also, best to only use GPS if we've got at least 4 sats active -- really should be like 5 or 6
		// Finally, it takes 3-5 secs of runtime for the gps heading to converge.
		useGps = (nowState.gpsSats > 4 &&
				nowState.lrEncSpeed > config.gpsValidSpeed &&
				nowState.rrEncSpeed > config.gpsValidSpeed &&
				(thisTime-timeZero) > 3000);// gps hdg converges by 3-5 sec.
#endif
	}

	useGpsStat = useGps;

#if 0

	//////////////////////////////////////////////////////////////////////////////
	// HEADING AND POSITION UPDATE
	//////////////////////////////////////////////////////////////////////////////

	// TODO: 3 Position filtering
	//    position will be updated based on heading error from heading estimate
	// TODO: 3 Distance/speed filtering
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
	history[now].dist = (sensors.lrEncDistance + sensors.rrEncDistance) / 2.0;// current distance traveled
	history[now].gyro = sensors.gyro[_z_];// current raw gyro data
	history[now].dt = dt;// current dt, to address jitter
	history[now].hdg = clamp360( history[prev].hdg + dt*(history[now].gyro) );// compute current heading from current gyro
	float r = PI/180 * history[now].hdg;
	history[now].x = history[prev].x + history[now].dist * sin(r);// update current position
	history[now].y = history[prev].y + history[now].dist * cos(r);

	// We can't do anything until the history buffer is full
	if (hCount < sensors.gps.lag) {
		estLagHeading = history[now].hdg;
		// Until the fifo is full, only keep track of current gyro heading
		hCount++;// after n iterations the fifo will be full
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
		if (go) {
			estLagHeading = headingKalman(history[lag].dt, nowState.gpsCourse_deg, useGps, history[lag].gyro, true);
		} else {
			estLagHeading = headingKalman(history[lag].dt, initialHeading, true, history[lag].gyro, true);
		}

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
		errHeading = clamp180(estLagHeading - history[lag].hdg);

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
	here.set(history[now].x, history[now].y);
#endif

	//////////////////////////////////////////////////////////////////////////////
	// NAVIGATION UPDATE
	//////////////////////////////////////////////////////////////////////////////

	bearing = here.bearingTo(config.cwpt[nextWaypoint]);
	distance = here.distanceTo(config.cwpt[nextWaypoint]);

	// if within config.waypointDist distance threshold move to next waypoint
	// TODO 3 figure out how to output feedback on wpt arrival external to this function
	if (go) {

#if 0
		// if we're within brakeDist of next or previous waypoint, run @ turn speed
		// This would normally mean we run at turn speed until we're brakeDist away
		// from waypoint 0, but we trick the algorithm by initializing prevWaypoint to waypoint 1
		// TODO 2 make this distance related -- if (here.distanceTo(config.cwpt[lastWaypoint] < 0.5 ) {
		if ((thisTime - timeZero) < 3000) {
			setSpeed(config.startSpeed);
		} else if (distance < config.brakeDist
				|| prevDistance < config.brakeDist) {
			setSpeed(config.turnSpeed);
			// TODO 3 setSpeed( config.wptTurnSpeed[nextWaypoint] );
		} else {
			setSpeed(config.topSpeed);
			// TODO 3 setSpeed( config.wptTopSpeed[nextWaypoint] );
		}
#endif

		if (distance < config.waypointDist) {
			//fprintf(stdout, "Arrived at wpt %d\n", nextWaypoint);
			//speaker.beep(3000.0, 0.5); // non-blocking
			lastWaypoint = nextWaypoint;
			nextWaypoint++;
		}

	} else {
		setSpeed(0.0);
	}
	// Are we at the last waypoint?
	// currently handled external to this routine

	//////////////////////////////////////////////////////////////////////////////
	// OBSTACLE DETECTION & AVOIDANCE
	//////////////////////////////////////////////////////////////////////////////
	// TODO 4 limit steering angle based on object detection ? or limit relative brg perhaps?
	// TODO 4 add vision obstacle detection and filtering
	// TODO 4 add ranger obstacle detection and filtering/fusion with vision

	// For Steering Angle Computation below
	static float relBrg;
	static CartPosition LA;

	//////////////////////////////////////////////////////////////////////////////
	// CONTROL UPDATE
	//////////////////////////////////////////////////////////////////////////////

	if (--control_count == 0) {

		//////////////////////////////////////////////////////////////////////////////////////
		// Steering Control
		//
		float sign = 1;

#if PURE_PURSUIT
		static CartPosition A, C;

		// Update the A and C points
		A = config.cwpt[lastWaypoint];
		C = config.cwpt[nextWaypoint];

		// Leg vector
		float Lx = C.x - A.x;
		float Ly = C.y - A.y;
		// Robot vector
		float Rx = here.x - A.x;
		float Ry = here.y - A.y;

		// Find the goal point, a projection of the bot vector onto the current leg, moved
		// along the path by the lookahead distance.
		float legLength = sqrtf(Lx * Lx + Ly * Ly); // ||L||
		float proj = (Lx * Rx + Ly * Ry) / legLength; // R dot L/||L||, projection magnitude, bot vector onto leg vector
		// Now find projection point + lookahead, along leg, relative to A.
		LA.set(A.x + (proj + config.intercept) * Lx / legLength,
				A.y + (proj + config.intercept) * Ly / legLength);
#else
		LA.set(config.cwpt[nextWaypoint]);
#endif
		//
		// Compute a circle that is tangential to bot heading and intercepts bot
		// and goal point LA, the intercept circle. Then compute the steering
		// angle to trace that circle.
		//
		// First, compute absolute bearing to lookahead point (LA) from robot (B)
		float brg = here.bearingTo(LA);
		// Now, compute relative bearing to the lookahead. Relative to bot hdg.
		relBrg = clamp180(brg - heading);
//        fprintf(stdout, "ih=%.1f hdg=%.1f brg=%.1f rel=%.1f\n", initialHeading, hdg, brg, relBrg);
//        fprintf(stdout, "gyro=%3d\n", sensors.g[2]);
		// The steering angle equation actually peaks at relBrg == 90 so just clamp to this
		// Also, we get a div-by-zero if relBrg == 0 so we don't even correct steering
		// if it's < some minimum
		if (relBrg > 89.5) {
			relBrg = 89.5;
		} else if (relBrg < -89.5) {
			relBrg = -89.5;
		} else if (fabs(relBrg) > 0.0005) { // avoid div-by-zero
			// Compute turn radius based on intercept distance and specified angle
			// The sin equation will produce a negative radius, which causes problems
			// when subtracting track/2.0, so just take absolute value and multiply sign
			// later on
			sign = (relBrg < 0) ? -1 : 1;
			float radius = config.intercept / fabs(2 * sin(Steering::toRadians(relBrg)));
			// optionally, limit radius min/max
			// Now compute the steering angle to achieve the circle of
			// Steering angle is based on wheelbase and track width
			steerAngle = sign * Steering::toDegrees(asin(config.wheelbase / (radius - config.track / 2.0)));
			// Apply gain factor for near straight line
			// TODO 3 figure out a better, continuous way to deal with steering gain
			//        if (fabs(steerAngle) < config.steerGainAngle) steerAngle *= config.steerGain;
			steering = steerAngle;
		}
//        timeB = timer.read_us();

		//
		//////////////////////////////////////////////////////////////////////////////////////

#if 0
		//////////////////////////////////////////////////////////////////////////////////////
		// Throttle PID Control
		//
		// TODO: 3 move all this PID crap into Actuators.cpp
		// TODO: 3 probably should do KF or something for speed/dist; need to address GPS lag, too
		// if nothing else, at least average the encoder speed over mult. samples
		if (desiredSpeed <= 0.1) {
			esc = config.escMin;
		} else {
			// PID loop for throttle control
			// http://www.codeproject.com/Articles/36459/PID-process-control-a-Cruise-Control-example
			float error = desiredSpeed - nowSpeed;
			// Keep track of accumulated error, but only if we're not sitting still, due to
			// being at the line with manual control enabled.
			if (nowSpeed > 0.5)
				integral += (error * speedDt);
			// determine the amount of change from the last time checked
			float derivative = (error - lastError) / speedDt;
			// calculate how much to drive the output in order to get to the
			// desired setpoint.
			float output = config.escZero + (config.speedKp * error)
					+ (config.speedKi * integral)
					+ (config.speedKd * derivative);
			if (output > config.escMax)
				output = config.escMax;
			if (output < config.escZero)
				output = config.escZero;
//            fprintf(stdout, "s=%.1f d=%.1f o=%.1f\n", nowSpeed, desiredSpeed, output);
			esc = (int) output;
			// remember the error for the next time around so we can compute delta error.
			lastError = error;
		}

		speedDt = 0; // reset dt to begin counting for next time
#endif

		control_count = CTRL_SKIP;
	}

	//////////////////////////////////////////////////////////////////////////////
	// DATA FOR LOGGING
	//////////////////////////////////////////////////////////////////////////////

	// Log Data Timestamp
	//nowState.millis = thisTime;
	nowState.millis = thisTime;
	nowState.voltage = 0.0; // unused
	nowState.current = 0.0; // unused
	for (int i = 0; i < 3; i++) {
		nowState.m[i] = 0.0; // unused
		nowState.g[i] = 0.0; // unused
		nowState.gyro[i] = 0.0; // unused
		nowState.a[i] = 0.0; // unused
	}
	nowState.gTemp = 0.0; // unused
	nowState.lrEncSpeed = nowState.rrEncSpeed = nowState.gpsSpeed_mps;
	nowState.lrEncDistance = 0.0; // unused
	nowState.rrEncDistance = 0.0; // unused
	//state.encHeading += (state.lrEncDistance - state.rrEncDistance) / TRACK;
	nowState.estHeading = heading;
	nowState.estLagHeading = 0.0; // unused
	nowState.estLatitude = hereGeo.latitude();
	nowState.estLongitude = hereGeo.longitude();
	nowState.estX = here.x;
	nowState.estY = here.y;
	nowState.bearing = bearing;
	nowState.distance = distance;
	nowState.nextWaypoint = nextWaypoint;
	//nowState.gbias = 0;
	nowState.errHeading = 0.0; // unused
	//state.leftRanger = sensors.leftRanger;
	//state.rightRanger = sensors.rightRanger;
	//state.centerRanger = sensors.centerRanger;
	nowState.steerAngle = steerAngle;
	nowState.LABrg = relBrg;
	nowState.LAx = LA.x;
	nowState.LAy = LA.y;

	// Periodically, we enter a new SystemState into the FIFO buffer
	// The main loop handles logging and will catch up to us provided
	// we feed in new log entries slowly enough.
	if (--log_count == 0) {
		fifo_push(&nowState);
//		state_clear(&nowState);
		log_count = LOG_SKIP;       // reset counter
	}

#if 0
	// increment history fifo pointers with wrap-around
	prev = now;
	inc(now);
#endif

	// timing
	tReal = timer.read_us() - tReal;
}
