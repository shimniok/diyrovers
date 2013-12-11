#ifndef __INCREMENTALENCODER_H
#define __INCREMENTALENCODER_H

#include "mbed.h"

/** An interface for a simple, 1-track, incremental encoder. If using a simple reflectance sensor, then a voltage comparator
 *  circuit will be required to generate the pulsetrain.  See: http://www.bot-thoughts.com/2011/03/avc-bot-wheel-encoders.html
 *
 */
class IncrementalEncoder
{
    public:
        /** Create an incremental encoder interface.  Increments counter at every rise and fall signal 
         *
         * @param pin -- the pin to which a digital pulsetrain is sent
         */
        IncrementalEncoder(PinName pin);

        /** Get ticks since last call
         *
         * @returns the number of ticks since the last call to this method
         */        
        unsigned int read();

        /** Get total tick count since last reset
         *
         * @returns total ticks since the last reset or instantiation
         */
        unsigned int readTotal();
       
        /** Get total rise tick count
         *
         * @returns total rise ticks
         */
        unsigned int readRise();

        /** Get total fall tick count
         *
         * @returns total fall ticks
         */
        unsigned int readFall();

        /** Read time interval between ticks
         *
         * @returns filtered time between encoder pulses
         */
        unsigned int readTime();
       
        /** Reset the tick counter
         *
         */
        void reset();

    private:
        Timer _t;
        unsigned int _lastTime;
        unsigned int _time;
        unsigned int _lastTicks;
        unsigned int _ticks, _rise, _fall;
        bool _new;
        InterruptIn _interrupt;
        void _increment();
        void _incRise();
        void _incFall();
};

#endif