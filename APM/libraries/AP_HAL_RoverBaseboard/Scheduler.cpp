
#include "Scheduler.h"

using namespace RoverBaseboard;

extern const AP_HAL::HAL& hal;

RoverBaseboardScheduler::RoverBaseboardScheduler()
{}

void RoverBaseboardScheduler::init(void* machtnichts)
{}

void RoverBaseboardScheduler::delay(uint16_t ms)
{}

uint32_t RoverBaseboardScheduler::millis() {
    return 10000;
}

uint32_t RoverBaseboardScheduler::micros() {
    return 200000;
}

void RoverBaseboardScheduler::delay_microseconds(uint16_t us)
{}

void RoverBaseboardScheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{}

void RoverBaseboardScheduler::register_timer_process(AP_HAL::MemberProc k)
{}

void RoverBaseboardScheduler::register_io_process(AP_HAL::MemberProc k)
{}

void RoverBaseboardScheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

void RoverBaseboardScheduler::suspend_timer_procs()
{}

void RoverBaseboardScheduler::resume_timer_procs()
{}

bool RoverBaseboardScheduler::in_timerprocess() {
    return false;
}

void RoverBaseboardScheduler::begin_atomic()
{}

void RoverBaseboardScheduler::end_atomic()
{}

bool RoverBaseboardScheduler::system_initializing() {
    return false;
}

void RoverBaseboardScheduler::system_initialized()
{}

void RoverBaseboardScheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

void RoverBaseboardScheduler::reboot(bool hold_in_bootloader) {
    for(;;);
}
