/*
 * Servo2.cpp
 *
 *  Created on: May 21, 2014
 *      Author: mes
 */

#include "Servo2.h"


Servo2::Servo2(PinName pin):
	_pwm(pin)
{
	_pwm.period_us(20000);
    _pwm.pulsewidth_us(1500);
}


void Servo2::period(int microseconds)
{
    _pwm.period_us(microseconds);
}

void Servo2::write(int microseconds)
{
    _pwm.pulsewidth_us(microseconds);
}

int Servo2::read() {
	return _pwm.read();
}

int Servo2::operator =(int microseconds)
{
    write(microseconds);
    return microseconds;
}

Servo2::operator float()
{
    return read();
}
