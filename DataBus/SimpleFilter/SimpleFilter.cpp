#include "SimpleFilter.h"

SimpleFilter::SimpleFilter(short shift): _filter_value(0), _shift(shift) {
    // nothing to do here, really
}

short SimpleFilter::filter(short value) {

    _filter_value += (value - (_filter_value >> _shift));
    
    return _filter_value >> _shift;
}

short SimpleFilter::value(void) {
    return _filter_value >> _shift;
}
