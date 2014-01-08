#include "mbed.h"
#include "Servo.h"
#include "Config.h"

// Measure wheel angle of both wheels at various servo values.
// There should be a near linear relationship between servo value and steering angle
// up to about 20 degrees.
// Now find values of middle and scale that describe this linear relationship.
// I came up with 0.5 and 1/808.0 (0.001238) on my ECX Circuit.
// Enter those values into the config.

Servo steering(p21);                    // Steering Servo
Servo throttle(p22);                    // ESC
float escMin=0.400;						// Servo setting for minimum (brake) throttle
float escMax=0.520;                     // Servo setting for max throttle
float escZero=0.400;					// Setting for zero speed (middle value)
float escScale=1.0;						// Scales throttle specification to servo value
float steerZero=0.500;					// Middle value for servo (wheels straight)
float steerScale=808.0;					// Scales steering angle to servo value

// TODO: 2 rework servo library to use standard ms value

void setSteering(float steerAngle) {
    // Convert steerAngle to servo value assuming linear relationship between steering angle
	// and servo value.
    //
    steering = steerZero + steerAngle * steerScale;
}

float getSteering(void) {
	return steering.read();
}

// Setup servo outputs
void initSteering() {
	setSteering(0);
    steering.calibrate(0.005, 45.0);
}

void setSteerMiddle(float m) {
	steerZero = m;
	return;
}

void setSteerScale(float s) {
	steerScale = s;
	return;
}

void setThrottle(float value)
{
	if (value > escMax) value = escMax;
	if (value < escMin) value = escMin;
	throttle = value * escScale;

	return;
}

float getThrottle(void) {
	return throttle.read();
}

void initThrottle()
{
	setThrottle(0);
    //throttle.calibrate(5.0/1000.0, 45.0);
}

void setThrottleMiddle(float m) {
	escZero = m;
	return;
}

void setThrottleMin(float m) {
	escMin = m;
}

void setThrottleMax(float m) {
	escMax = m;
	return;
}

void setThrottleScale(float s) {
	escScale = s;
	return;
}
