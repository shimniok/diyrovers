#ifndef __ACTUATORS_H
#define __ACTUATORS_H

/** Abstraction of steering and throttle control */

/** set steering to steering angle */
void setSteering(float steerAngle);

/** get actual servo setting */
float getSteering(void);

/** initialize steering */
void initSteering(void);

/** calibrate with middle servo value
 *
 * servoValue = steerAngle*scale + middle;
 */
void setSteerMiddle(float m);

/** calibrate scaling of steering angle to servo value
 *
 * servoValue = steerAngle*scale + middle;
 */
void setSteerScale(float s);

/** set throttle to throttle value */
void setThrottle(float value);

/** return actual servo setting */
float getThrottle(void);

/** initialize throttle */
void initThrottle(void);

/** calibrate throttle middle (off) value */
void setThrottleMiddle(float m);

/** calibrate throttle minimum (brake) value */
void setThrottleMin(float m);

/** calibrate throttle maximum value */
void setThrottleMax(float m);

/** calibrate scaling of throttle value to servo value */
void setThrottleScale(float s);

#endif
