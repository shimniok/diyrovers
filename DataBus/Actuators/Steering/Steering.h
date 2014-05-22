#ifndef __STEERING_H
#define __STEERING_H

#include "globals.h"
#include "devices.h"
#include "Servo.h"
#include "Config.h"

/** A class for managing steering angle calculations based on current and desired heading
 *  and specified intercept distance along the new path.  
 *
 *  See Notebook entry: http://mbed.org/users/shimniok/notebook/smooth-steering-for-rc-car/
 */
class Steering
{
  public:
    /** create a new steering calculator */
	Steering(PinName pin);

	/** Initalize steering actuator (servo)
	 *
	 */
	void initSteering(void);

    /** Set the track width (left to right contact patch distance)
     *
     * @param track is the vehicle track width
     */
    void setTrack(float track);

    /** Set the wheelbase (front to rear axle distance)
     *
     * @param wheelbase is the vehicle wheelbase
     */
    void setWheelbase(float wheelbase);

    /** set intercept distance
     * @param intercept distance along new course at which turn arc will intercept
     */
    void setIntercept(float intercept);


    /** Convert steerAngle to servo value
     *
     * Testing determined near linear conversion between servo ms setting and steering angle
     * up to ~20*.
     *
     * @param steerAngle is the steering angle, measured at front wheels, averaged
     */
    void setSteering(float steerAngle);


    /**  Shorthand for the write function */
    float operator= (float steerAngle);


    /** convert course change to average steering angle
     * assumes Ackerman steering, with track and wheelbase
     * and course intercept distance specified.
     *
     * See notebook: http://mbed.org/users/shimniok/notebook/smooth-steering-for-rc-car/
     *
     * @param theta relative bearing of the new course
     * @returns steering angle in degrees
     */ 
    float calcSA(float theta);


    /** convert course change to average steering angle
     * assumes Ackerman steering, with track and wheelbase
     * and course intercept distance specified. Also, |radius| of turn is limited to limit
     *
     * See notebook: http://mbed.org/users/shimniok/notebook/smooth-steering-for-rc-car/
     *
     * @param theta relative bearing of the new course
     * @param limit is the limit of the turn circle radius (absolute value)
     * @returns steering angle in degrees
     */ 
    float calcSA(float theta, float limit);


    /** compute steering angle based on pure pursuit algorithm
     */
    float purePursuitSA(float hdg, float Bx, float By, float Ax, float Ay, float Cx, float Cy);
    

    /** compute steering angle based on a simpler path pursuit variant of pure pursuit
     */
    float pathPursuitSA(float hdg, float Bx, float By, float Ax, float Ay, float Cx, float Cy);


    /** Compute cross track error given last waypoint, next waypoint, and robot coordinates
     * @returns cross track error
     */
    float crossTrack(float Bx, float By, float Ax, float Ay, float Cx, float Cy);


    /** Convert degrees to radians
     * @param deg degrees to convert
     * @return radians
     */
    inline static float toRadians(float deg) {return (PI/180.0)*deg;}


    /** Convert radians to degrees
     * @param rad radians to convert
     * @return degrees
     */
    inline static float toDegrees(float rad) {return (180/PI)*rad;}

  private:
    Servo _steering;               // Steering Servo
};

#endif
