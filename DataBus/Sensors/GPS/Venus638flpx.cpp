#include "mbed.h"
#include "Venus638flpx.h"

// TODO: parameterize LED

Venus638flpx::Venus638flpx(PinName tx, PinName rx):
    serial(tx, rx)
{
    init();
}

void Venus638flpx::init()
{
    setBaud(38400);
    enable();
    disableVerbose();
    setUpdateRate(10);
}

void Venus638flpx::setBaud(int baud)
{
    serial.baud(baud);
}

Serial *Venus638flpx::getSerial(void)
{
    return &serial;
}

void Venus638flpx::enable(void)
{
    reset_available();
    serial.attach(this, &Venus638flpx::recv_handler, Serial::RxIrq);
}

void Venus638flpx::disable(void)
{
    serial.attach(NULL, Serial::RxIrq);
}

/**
 * Enable verbose messages for debugging
 */
void Venus638flpx::enableVerbose(void)
{
    setNmeaMessages(1,1,1,0,1,1);
}

/**
 * Disable verbose messages for debugging
 */
void Venus638flpx::disableVerbose(void)
{
    setNmeaMessages(1,1,0,0,1,0);
}

void Venus638flpx::setNmeaMessages(char gga, char gsa, char gsv, char gll, char rmc, char vtg)
{
    // VENUS Binary MsgID=0x08
    // GGA interval
    // GSA interval
    // GSV interval
    // GLL interval
    // RMC interval
    // VTG interval
    // ZDA interval -- hardcode off
    char msg[15] = { 0xA0, 0xA1, 0x00, 0x09,
                     0x08, gga, gsa, gsv, gll, rmc, vtg, 0,
                     0, 0x0D, 0x0A
                   };
    for (int i=4; i < 12; i++) {
        msg[12] ^= msg[i];
    }
    for (int i=0; i < 15; i++)
        serial.putc(msg[i]);
}

void Venus638flpx::setUpdateRate(int rate)
{
    char msg[10] = {0xA0,
    				0xA1,
    				0x00,
    				0x03,
    				0x0E,
    				0,	// rate
    				01,
    				0,
    				0x0D,
    				0x0A
                   };

    msg[5] = rate&0xff;

    for (int i=4; i < 7; i++) {
        msg[7] ^= msg[i];
    }
    switch (rate) {
        case 1 :
        case 2 :
        case 4 :
        case 5 :
        case 8 :
        case 10 :
        case 20 :
            for (int i=0; i < 10; i++)
                serial.putc(msg[i]);
            break;
        default :
            break;
    }
}


/*
void Venus638flpx::position(GeoPosition &here)
{
    double latitude, longitude;
    unsigned long age;
    nmea.f_get_position(&latitude, &longitude, &age);
    here.set(latitude, longitude);
    return;
}

void Venus638flpx::position(double *latitude, double *longitude)
{
    unsigned long age;
    nmea.f_get_position(latitude, longitude, &age);
    return;
}
*/

double Venus638flpx::latitude(void)
{
    double latitude, longitude;
    unsigned long age;
    nmea.f_get_position(&latitude, &longitude, &age);
    return latitude;
}

double Venus638flpx::longitude(void)
{
    double latitude, longitude;
    unsigned long age;
    nmea.f_get_position(&latitude, &longitude, &age);
    return longitude;
}

float Venus638flpx::hdop(void)
{
    return nmea.f_hdop();
}

int Venus638flpx::sat_count(void)
{
    return nmea.sat_count();
}

float Venus638flpx::speed_mps(void)
{
    return nmea.f_speed_mps();
}

float Venus638flpx::heading_deg(void)
{
    return nmea.f_course();
}

bool Venus638flpx::available(void)
{
    return nmea.ready();
}

void Venus638flpx::reset_available(void)
{
    nmea.reset_ready();
}

int Venus638flpx::getAvailable(void)
{
    // TODO 2 not sure what to do here
    return 0xff;
}

void Venus638flpx::recv_handler()
{
    while (serial.readable()) {
        nmea.encode(serial.getc());
    }
}
