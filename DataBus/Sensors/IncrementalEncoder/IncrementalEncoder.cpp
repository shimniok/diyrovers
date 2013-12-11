#include "IncrementalEncoder.h"

IncrementalEncoder::IncrementalEncoder(PinName pin):  _lastTicks(0),  _ticks(0), _new(false), _interrupt(pin) {
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
    _time = A*_time + (1-A)*(now - _lastTime);
    // Dead band: if _time > xxxx then turn off and wait until next tick
    // to start up again
    _lastTime = now;
}

void IncrementalEncoder::_incFall() {
    _fall++;
    _ticks++;
    _new=true;
}