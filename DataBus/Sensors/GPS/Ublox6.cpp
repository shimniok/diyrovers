#include "mbed.h"
#include "Ublox6.h"

#define DOP_BIT     0x01
#define POSLLH_BIT  0x02
#define SOL_BIT     0x04
#define VELNED_BIT  0x08

// TODO 2: parameterize LED/status

Ublox6::Ublox6(PinName tx, PinName rx):
    serial(tx, rx),
    _available(false)
{
    init();
}

void Ublox6::init(void)
{
    setBaud(38400);
    enable();
    disableVerbose();
}


Serial *Ublox6::getSerial(void)
{
    return &serial;
}

void Ublox6::setBaud(int baud)
{
    serial.baud(baud);
}

void Ublox6::enable(void)
{
    reset_available();
    serial.attach(this, &Ublox6::recv_handler, Serial::RxIrq);
}

void Ublox6::disable(void)
{
    serial.attach(NULL, Serial::RxIrq);
}

/**
 * Enable verbose messages for debugging
 */
void Ublox6::enableVerbose(void)
{
    // TODO: enable Verbose code
}

/**
 * Disable verbose messages for debugging
 */
void Ublox6::disableVerbose(void)
{
    // TODO: disable Verbose code
}

bool Ublox6::available(void)
{
    return (_available & (DOP_BIT|POSLLH_BIT|SOL_BIT|VELNED_BIT));
}

void Ublox6::reset_available(void)
{
    //led2 = !led2;
    _available = 0;
}

double Ublox6::latitude(void)
{
    return _latitude;
}

double Ublox6::longitude(void)
{
    return _longitude;
}

float Ublox6::speed_mps(void)
{
    return _speed_mps;
}

float Ublox6::heading_deg(void)
{
    return _course_deg;
}

float Ublox6::hdop(void)
{
    return _hdop;
}

int Ublox6::sat_count(void)
{
    return _sat_count;
}

void Ublox6::parse(unsigned char cc)
{
    //unsigned char cc = buf[out++];
    //out &= (MAX_LENGTH-1);
    static unsigned char ck1, ck2, state, code, id, idx, length, chk1, chk2;
    static bool checkOk;
    static unsigned char data[MAX_LENGTH];

    switch (state) {
        case 0:    // wait for sync 1 (0xB5)
            ck1 = ck2 = 0;
            checkOk = false;
            if (cc == SYNC1)
                state++;
            break;
        case 1:    // wait for sync 2 (0x62)
            if (cc == SYNC2)
                state++;
            else
                state = 0;
            break;
        case 2:    // wait for class code
            code = cc;
            ck1 += cc;
            ck2 += ck1;
            state++;
            break;
        case 3:    // wait for Id
            id = cc;
            ck1 += cc;
            ck2 += ck1;
            state++;
            break;
        case 4:    // wait for length uint8_t 1
            length = cc;
            ck1 += cc;
            ck2 += ck1;
            state++;
            break;
        case 5:    // wait for length uint8_t 2
            length |= (unsigned int) cc << 8;
            ck1 += cc;
            ck2 += ck1;
            idx = 0;
            state++;
            if (length > MAX_LENGTH)
                state= 0;
            break;
        case 6:    // wait for <length> payload uint8_ts
            data[idx++] = cc;
            ck1 += cc;
            ck2 += ck1;
            if (idx >= length) {
                state++;
            }
            break;
        case 7:    // wait for checksum 1
            chk1 = cc;
            state++;
            break;
        case 8: {  // wait for checksum 2
            chk2 = cc;
            checkOk = ck1 == chk1  &&  ck2 == chk2;
            if (!checkOk) {
                // do something...?
            } else {
                switch (code) {
                    case 0x01:      // NAV-
                        switch (id) {
                            case POSLLH_MSG:  // NAV-POSLLH
                                _longitude = ((float)LONG(4))/10000000.0;
                                _latitude = ((float)LONG(8))/10000000.0;
                                // vAcc = ULONG(24); // mm
                                // hAcc = ULONG(20); // mm
                                _available |= POSLLH_BIT;
                                break;
                            case DOP_MSG:  // NAV-DOP
                                //gDOP = ((float) UINT(4))/100.0;
                                //tDOP = ((float) UINT(8))/100.0;
                                //vDOP = ((float) UINT(10))/100.0;
                                _hdop = ((float) UINT(12))/100.0;
                                _available |= DOP_BIT;
                                break;
                            case SOL_MSG:  // NAV-SOL
                                //week = UINT(8);
                                //pDOP = ((float) UINT(44))/ 100.0;
                                //pAcc = ULONG(24);
                                _sat_count = data[47];
                                _available |= SOL_BIT;
                                break;
                            case VELNED_MSG:  // NAV-VELNED
                                _speed_mps = ULONG(20)/100.0;
                                //sAcc = ULONG(28)/100.0;
                                _course_deg = ((float) LONG(24))/100000.0;
                                //cAcc = ((float) LONG(32))/100000.0;
                                _available |= VELNED_BIT;                                
                                break;
                            default:
                                break;
                        }
                        break;
                    case 0x05:      // ACK-
                        switch (id) {
                            case 0x00:  // ACK-NAK
                                break;
                            case 0x01:  // ACK-ACK
                                break;
                        }
                        break;
                }
            }
            state = 0;
            break;
        }
        default:
            break;
    }
}


void Ublox6::recv_handler(void)
{
    while (serial.readable())
        parse(serial.getc());
}