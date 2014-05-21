#include "IncrementalEncoder.h"

// TODO 2: incorporate distance calcs and config setting

IncrementalEncoder::IncrementalEncoder(PinName pin)
:	_new(false)
,	_lastTime(0)
,	_time(0)
,   _lastTicks(0)
,	_ticks(0)
,	_rise(0)
,	_fall(0)
,	_interrupt(pin)
{
    _interrupt.mode(PullNone); // default is pulldown but my encoder board uses a pull-up and that just don't work
    _interrupt.rise(this, &IncrementalEncoder::_incRise); 
    _interrupt.fall(this, &IncrementalEncoder::_incFall); 
    _t.start();
    _t.reset();
    _lastTime = _t.read_us();
}

unsigned int IncrementalEncoder::read() {
// disable interrupts?
    unsigned int ticks = _ticks - _lastTicks;
    _lastTicks = _ticks;
    _new=false;
    return ticks;
}  

unsigned int IncrementalEncoder::readTotal() {
    _new=false;
    return _ticks;
}

unsigned int IncrementalEncoder::readRise() {
    _new=false;
    return _rise;
}  

unsigned int IncrementalEncoder::readFall() {
    _new=false;
    return _fall;
}
    
unsigned int IncrementalEncoder::readTime() {
    return _time;
}
    
void IncrementalEncoder::reset() {
    _ticks = _lastTicks = 0;
}  

void IncrementalEncoder::_increment() {
    _ticks++;
}

#define A 0.3 // mix in how much from previous readings?
void IncrementalEncoder::_incRise() {
    _rise++;
    _ticks++;
    _new=true;
    // compute time between ticks; only do this for rise
    // to eliminate jitter
    int now = _t.read_us();
    _time = A*_time + (1-A)*(now - _lastTime); // FIXME bug with speed after stopping
    // Dead band: if _time > xxxx then turn off and wait until next tick
    // to start up again
    _lastTime = now;
}

void IncrementalEncoder::_incFall() {
    _fall++;
    _ticks++;
    _new=true;
}
