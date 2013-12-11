
#include "Semaphores.h"

using namespace RoverBaseboard;

bool RoverBaseboardSemaphore::give() {
    if (_taken) {
        _taken = false;
        return true;
    } else {
        return false;
    }
}

bool RoverBaseboardSemaphore::take(uint32_t timeout_ms) {
    return take_nonblocking();
}

bool RoverBaseboardSemaphore::take_nonblocking() {
    /* No syncronisation primitives to garuntee this is correct */
    if (!_taken) {
        _taken = true;
        return true;
    } else {
        return false;
    }
}
