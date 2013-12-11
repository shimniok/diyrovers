#include "mbed.h"
#include "Beep.h"

/** Create a Beep object connected to the specified PwmOut pin
 *
 * @param pin PwmOut pin to connect to 
 */
Beep::Beep(PinName pin) : _pwm(pin) {
    _pwm.write(0.0);     // after creating it have to be off
}

 /** Stop the beep instantaneously.
  */
void Beep::nobeep() {
    _pwm.write(0.0);
}

/** Beep with given frequency and duration.
 *
 * @param frequency - the frequency of the tone in Hz
 * @param time - the duration of the tone in seconds
 */
     
void Beep::beep(float freq, float time) {

    _pwm.period(1.0/freq);
    _pwm.write(0.5);            // 50% duty cycle - beep on
    toff.attach(this,&Beep::nobeep, time);   // time to off
}




