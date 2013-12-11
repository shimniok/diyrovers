#ifndef MBED_BEEP_H
#define MBED_BEEP_H

#include "mbed.h"

/** Generates a tone with a buzzer, based on a PwmOut
 * The class use a timeout to switch off the sound  - it is not blocking while making noise
 *
 * Example:
 * @code
 * // Beep at 2kHz for 0.5 seconds
 * #include "mbed.h"
 * #include "Beep.h"
 * 
 * Beep buzzer(p21);
 * 
 * int main() {
 *        ...
 *   buzzer.beep(2000,0.5);    
 *       ...
 * }
 * @endcode
 */
class Beep {

public:

/** Create a Beep object connected to the specified PwmOut pin
 *
 * @param pin PwmOut pin to connect to 
 */
    Beep(PinName pin);

/** Beep with given frequency and duration.
 *
 * @param frequency - the frequency of the tone in Hz
 * @param time - the duration of the tone in seconds
 */
    void beep(float frequency, float time);

/** stop the beep instantaneously. Not typically needed, but here just in case
 */
    void nobeep();

private :
    PwmOut _pwm;
    Timeout toff;
};
#endif
