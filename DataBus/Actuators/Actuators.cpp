#include "mbed.h"
#include "Servo.h"
#include "Config.h"

extern Config config;

Servo steering(p21);                    // Steering Servo
Servo throttle(p22);                    // ESC
int escMax=520;                         // Servo setting for max speed

#define THROTTLE_CENTER 0.4

void initSteering()
{
    if (config.loaded) {
        // Setup steering servo
        steering = config.steerZero;
    } else {
        steering = 0.4;
    }
    // TODO: 3 parameterize this in config file
    steering.calibrate(0.005, 45.0); 
}


void initThrottle()
{
    if (config.loaded) {
        throttle = (float)config.escZero/1000.0;
    } else {
        throttle = 0.410;
    }
    // TODO: 3 parameterize this in config file
    throttle.calibrate(0.001, 45.0); 
}

void setThrottle(int value)
{
    throttle = ((float)value)/1000.0;
}

void setSteering(float steerAngle)
{
    // Convert steerAngle to servo value
    // Testing determined near linear conversion between servo ms setting and steering angle
    // up to 20*.  Assumes a particular servo library with range = 0.005
    // In that case, f(SA) = servoPosition = 0.500 + SA/762.5
    // between 20 and 24* the slope is approximately 475
    // What if we ignore the linearity and just set to a max angle
    // also range is 0.535-0.460 --> slope = 800
    // steering = 0.500 + (double) steerAngle / 762.5;
    //
    // TODO: 1 parameterize through config
    steering = 0.500 + (double) steerAngle / 808.0;
}
    
