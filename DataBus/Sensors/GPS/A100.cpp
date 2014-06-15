#include "mbed.h"
#include "globals.h"
#include "A100.h"

// TODO 3 parameterize LED

const int A100::lag=50; // FIXME lag variable

A100::A100(PinName tx, PinName rx):
    serial(tx, rx)
{
    init();
}

void A100::init()
{
    setBaud(115200);
//    for (int i=0; i < 100; i++) {
//    	serial.puts("$JDIFF,WAAS\r\n");
//    	wait(0.2);
//    }
//    serial.puts("$JSHOW\r\n");
//    enable();
//    disableVerbose();
//    setUpdateRate(5);
//    serial.puts("$JASC,GPGGA,10\r\n");
//	wait(1);
//	serial.puts("$JASC,GPRMC,10\r\n");
//	wait(1);
//	serial.puts("$JASC,GPGSV,1\r\n");
//	wait(1);
}


void A100::setBaud(int baud)
{
    serial.baud(baud);
}

Serial *A100::getSerial(void)
{
    return &serial;
}

void A100::enable(void)
{
    reset_available();
    serial.attach(this, &A100::recv_handler, Serial::RxIrq);
}

void A100::disable(void)
{
    serial.attach(NULL, Serial::RxIrq);
}

/**
 * Enable verbose messages for debugging
 */
void A100::enableVerbose(void)
{
    setNmeaMessages(1,1,1,0,1,1);
}

/**
 * Disable verbose messages for debugging
 */
void A100::disableVerbose(void)
{
    setNmeaMessages(1,1,0,0,1,0);
}

void A100::setNmeaMessages(char gga, char gsa, char gsv, char gll, char rmc, char vtg)
{
#if 0
	if (gga) {
		serial.puts("$JASC,GPGGA,10\r\n");
	}
	if (gsa) {
		serial.puts("$JASC,GPGSA,10\r\n");
	}
	if (gsv) {
		serial.puts("$JASC,GPGSV,10\r\n");
	}
	if (gll) {
		serial.puts("$JASC,GPGLL,10\r\n");
	}
	if (rmc) {
		serial.puts("$JASC,GPRMC,5\r\n");
	}
	if (vtg) {
		serial.puts("$JASC,GPVTG,5\r\n");
	}
#endif
}

void A100::setUpdateRate(int rate)
{
	// FIXME setUpdateRate
}


/*
void A100::position(GeoPosition &here)
{
    double latitude, longitude;
    unsigned long age;
    nmea.f_get_position(&latitude, &longitude, &age);
    here.set(latitude, longitude);
    return;
}

void A100::position(double *latitude, double *longitude)
{
    unsigned long age;
    nmea.f_get_position(latitude, longitude, &age);
    return;
}
*/

double A100::latitude(void)
{
    double latitude, longitude;
    unsigned long age;
    nmea.f_get_position(&latitude, &longitude, &age);
    return latitude;
}

double A100::longitude(void)
{
    double latitude, longitude;
    unsigned long age;
    nmea.f_get_position(&latitude, &longitude, &age);
    return longitude;
}

float A100::hdop(void)
{
    return nmea.f_hdop();
}

int A100::sat_count(void)
{
    return nmea.sat_count();
}

float A100::speed_mps(void)
{
    return nmea.f_speed_mps();
}

float A100::heading_deg(void)
{
    return nmea.f_course();
}

bool A100::available(void)
{
    return nmea.ready();
}

void A100::reset_available(void)
{
    nmea.reset_ready();
}

int A100::getAvailable(void)
{
    // TODO 2 not sure what to do here
    return 0xff;
}

void A100::recv_handler()
{
    while (serial.readable()) {
        nmea.encode((char) serial.getc());
    }
}
