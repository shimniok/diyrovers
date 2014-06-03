#include "IncrementalEncoder.h"

IncrementalEncoder::IncrementalEncoder(PinName pin)
:	_lastTime(0)
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

int IncrementalEncoder::read() {
    int ticks = _ticks - _lastTicks;
    _lastTicks = _ticks;
    return ticks;
}  

int IncrementalEncoder::readTotal() {
    return _ticks;
}

int IncrementalEncoder::readRise() {
    return _rise;
}  

int IncrementalEncoder::readFall() {
    return _fall;
}
    
int IncrementalEncoder::readTime() {
	int result = _time;
	_time = _lastTime = -1;
    return result;
}
    
void IncrementalEncoder::reset() {
    _ticks = _lastTicks = 0;
}  

void IncrementalEncoder::_increment() {
    _ticks++;
}

void IncrementalEncoder::_incRise() {
    _rise++;
    _ticks++;
    if (_lastTime < 0) {
    	_time = _lastTime = _t.read_us();
    } else {
		// compute time between ticks; only do this for rise to eliminate jitter
		// TODO 3: reimplement filtering of _time
		int now = _t.read_us();
		_time = now - _lastTime;
		_lastTime = now;
    }
}

void IncrementalEncoder::_incFall() {
    _fall++;
    _ticks++;
}
