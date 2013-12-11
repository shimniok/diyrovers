#include "mbed.h"
#include "Sirf3.h"

// TODO: parameterize LED

Sirf3::Sirf3(PinName tx, PinName rx):
    serial(tx, rx)
{
    enable();
}

void Sirf3::init(void)
{
    disableVerbose();
}

void Sirf3::setBaud(int baud)
{
    serial.baud(baud);
}

Serial *Sirf3::getSerial(void)
{
    return &serial;
}

void Sirf3::enable(void)
{
    reset_available();
    setBaud(4800);
    serial.attach(this, &Sirf3::recv_handler, Serial::RxIrq);
}

void Sirf3::disable(void)
{
    setBaud(4800);
    serial.attach(NULL, Serial::RxIrq);
}

/**
 * Enable verbose messages for debugging
 */
void Sirf3::enableVerbose(void)
{
    setBaud(4800);
    gsaMessage(true);
    ggaMessage(true);
    gllMessage(false);
    gsvMessage(true);
    rmcMessage(true);
    vtgMessage(false);
}

/**
 * Disable verbose messages for debugging
 */
void Sirf3::disableVerbose(void)
{
    setBaud(4800);
    gsaMessage(true);
    ggaMessage(true);
    gllMessage(false);
    gsvMessage(false);
    rmcMessage(true);
    vtgMessage(false);
}

void Sirf3::setUpdateRate(int rate)
{
    // We're stuck at 1Hz
    return;
}

int Sirf3::getAvailable(void)
{
    int answer = 0x00;
    if (nmea.gga_ready()) answer |= 0x01;
    if (nmea.rmc_ready()) answer |= 0x02;
    return answer;
}

double Sirf3::latitude(void)
{
    double latitude, longitude;
    unsigned long age;
    nmea.f_get_position(&latitude, &longitude, &age);
    return latitude;
}

double Sirf3::longitude(void)
{
    double latitude, longitude;
    unsigned long age;
    nmea.f_get_position(&latitude, &longitude, &age);
    return longitude;
}

float Sirf3::hdop(void)
{
    return nmea.f_hdop();
}

int Sirf3::sat_count(void)
{
    return nmea.sat_count();
}

float Sirf3::speed_mps(void)
{
    return nmea.f_speed_mps();
}

float Sirf3::heading_deg(void)
{
    return nmea.f_course();
}

bool Sirf3::available(void)
{
    return nmea.ready();
}

void Sirf3::reset_available(void)
{
    nmea.reset_ready();
}

void Sirf3::recv_handler(void)
{
    while (serial.readable()) {
        nmea.encode(serial.getc());
    }
}

void Sirf3::ggaMessage(bool enable)
{
    if (enable) {
        serial.printf("$PSRF103,00,00,01,01*25\r\n");     // Enable GGA
    } else {
        serial.printf("$PSRF103,00,00,00,01*24\r\n");     // Disable GGA
    }

    return;
}

void Sirf3::gllMessage(bool enable)
{
    if (enable) {
        serial.printf("$PSRF103,01,00,01,01*24\r\n");     // Enable GLL
    } else {
        serial.printf("$PSRF103,01,00,00,01*25\r\n");     // Disable GLL
    }

    return;
}

void Sirf3::gsaMessage(bool enable)
{
    if (enable) {
        serial.printf("$PSRF103,02,00,01,01*27\r\n");     // Enable GSA
    } else {
        serial.printf("$PSRF103,02,00,00,01*26\r\n");     // Disable GSA
    }

    return;
}

void Sirf3::gsvMessage(bool enable)
{
    if (enable) {
        serial.printf("$PSRF103,03,00,01,01*26\r\n");     // Enable GSV
    } else {
        serial.printf("$PSRF103,03,00,00,01*27\r\n");     // Disable GSV
    }

    return;
}

void Sirf3::rmcMessage(bool enable)
{
    if (enable) {
        serial.printf("$PSRF103,04,00,01,01*21\r\n");     // Enable RMC
    } else {
        serial.printf("$PSRF103,04,00,00,01*20\r\n");     // Disable RMC
    }

    return;
}

void Sirf3::vtgMessage(bool enable)
{
    if (enable) {
        serial.printf("$PSRF103,05,00,01,01*20\r\n");     // Enable VTG
    } else {
        serial.printf("$PSRF103,05,00,00,01*21\r\n");     // Disable VTG
    }

    return;
}
