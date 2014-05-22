/*
 * Servo2.h
 *
 *  Created on: May 21, 2014
 *      Author: mes
 */

#ifndef SERVO2_H_
#define SERVO2_H_

#include "stdint.h"
#include "mbed.h"

class Servo2 {
public:
	/** Create a new servo interface with common defaults
	 * Minimum = 600us, Maximum = 2400us
	 * @param pin is the servo signal pin
	 */
	Servo2(PinName sgnl);

	/** Set the signal period, default = 20000 us
	 *
	 * @param microseconds is the number of microseconds of signal period
	 */
	void period(int microseconds);

	/** Set the pulse width, default = 1500 us
	 *
	 * @param microseconds is the number of microseconds of puslse width
	 */
	void write(int microseconds);
	int operator =(int microseconds);

	/** Get the pulsewidth in microseconds
	 *
	 * @return pulsewidth in microseconds
	 */
	int read();
	operator float();

private:
	PwmOut _pwm;
};

#endif /* SERVO2_H_ */
