#ifndef __SCHEDULER_H
#define __SCHEDULER_H

/** Updater is the main real time sensor update, estimation, and control routine that is
 * called at 100Hz by a timer interrupt
 */

/** initialize throttle to center position */
void initThrottle(void);

/** attach the update routine to Ticker interrupt */
void startUpdater(void);

/** detach the update routine from Ticker interrupt */
void stopUpdater(void);

/** Returns the elapsed time taken by the updater routine on its most recent run */
int getUpdateTime(void);

/** Indicates to the updater that the vehicle should begin its run */
void beginRun(void);

/** Indicates to the updater that the vehicle should abort its run */
void endRun(void);

/** Tells the updater to re-initialize the navigation state */
void restartNav(void);

/** Sets the desired speed of the rover */
void setSpeed(float speed);

/** Sets the steering angle */
void setSteering(float steerAngle);

/** The function that is called at 100Hz. It reads sensors, performs estimation, and controls the robot */
void update(void);

#endif
