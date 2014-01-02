#include "Schedule.h"

Schedule::Schedule():
	_scale(0)
,	_max(0)
,	_clock(0)
,	_mode(0)
{
    for (int i=0; i < 64; i++) {
        _schedule[i] = 0;
    }
}

Schedule::Schedule(unsigned int scale, tick max, value start, value stop, flag m): _scale(scale), _max(max), _clock(0)
{
    Schedule::set(scale, max, start, stop, m);  
}

void Schedule::scale(unsigned int scale)
{
    _scale = scale;
}
        

void Schedule::max(tick max)
{
    if (_validTick(max))
        _max = max;
}


void Schedule::mode(flag m)
{
    _mode = m;
}


void Schedule::set(unsigned int scale, tick max, value start, value stop, flag m)
{
    if (_validTick(max)) {
        _scale = scale;
        _max = max;
        _mode = m;
        float slope = ((float) stop - (float) start) / (float) max;
        for (unsigned int i=0; i <= max; i++) {
            _schedule[i] = ((int) (slope*(float)i)) + start;
        }
    }
}


void Schedule::set(tick t, value v)
{
    if (_validTick(t)) {
        _schedule[t] = v;
        if (t > _max) _max = t;
    }
}      
  
      
value Schedule::get() 
{
    if (done()) {
        if (_mode == repeat)
            _clock %= _max+1;
        else if (_mode == hold)
            _clock = _max;
        else
            return 0;
    }
    
    return _schedule[_clock];
}      


value Schedule::next()
{
    _clock++;
    return get();
} 

                
bool Schedule::ticked(unsigned int time)
{
    bool result = false;
    
    if ((time % _scale) == 0) {
        _clock++;
        result = true;
    }
    
    return result;
}

bool Schedule::done()
{
    return (_clock > _max);
}        

bool Schedule::_validTick(tick t)
{
    return (t < 64); // unsigned int, always > 0; if wraps around will be > 64
}
