#ifndef __UBLOX6_H
#define __UBLOX6_H

/**
 * uBlox UBX Protocol Reader - Wayne Holder
 * Ported to mbed - Michael Shimniok
 *
 * Note: RX pad on 3DR Module is output, TX is input
 */
#include "mbed.h"
#include "GPS.h"

#define MAX_LENGTH 512

#define  SYNC1       0xB5
#define  SYNC2       0x62
#define  POSLLH_MSG  0x02
#define  SBAS_MSG    0x32
#define  VELNED_MSG  0x12
#define  STATUS_MSG  0x03
#define  SOL_MSG     0x06
#define  DOP_MSG     0x04
#define  DGPS_MSG    0x31
#define  SVINFO_MSG  0x30

#define CFG 0x06
#define NAV 0x01

#define MSG 0x01

#define LONG(X)    *(int32_t *)(&data[X])
#define ULONG(X)   *(uint32_t *)(&data[X])
#define INT(X)     *(int16_t *)(&data[X])
#define UINT(X)    *(uint16_t *)(&data[X])

class Ublox6: public GPS {
public:
    /**
     * create a new interface for Ublox6
     */
    Ublox6(PinName rx, PinName tx);

    /**
     * Initalize everything necessary for the GPS to collect the required data
     */
    virtual void init(void);
    
    /**
     * Set baud rate
     */    
    virtual void setBaud(int baud);

    /**
     * return serial object
     */
    virtual Serial *getSerial(void);

    /**
     * Disable serial data collection
     */    
    virtual void disable(void);

    /**
     * Enable serial data collection
     */
    virtual void enable(void);
    
    /**
    * Enable verbose messages for debugging
    */
    virtual void enableVerbose(void);

    /**
    * Disable verbose messages for debugging
    */    
    virtual void disableVerbose(void);

    /**
     * get latitude
     */    
    virtual double latitude(void);   
    
    /**
     * get longitude
     */
    virtual double longitude(void);

    /**
     * Get Horizontal Dilution of Precision
     * @return float horizontal dilution of precision
     */
    virtual float hdop(void);

    /**
     * get count of active satellites
     */
    virtual int sat_count(void);

    /**
     * get speed in m/s
     */    
    virtual float speed_mps(void);
    
    /**
     * get heading in degrees
     */
    virtual float heading_deg(void);
    
    /**
     * determine if data is available to be used
     */
    virtual bool available(void);
    
    /**
     * reset the data available flag
     */
    virtual void reset_available(void);

    int getAvailable(void) { return _available; }

    void setUpdateRate(int rate) { return; }

private:
    /**
     * Configure which Nmea messages to enable/disable
     */
    void recv_handler();

    /**
     * UBX protocol parser (Wayne Holder)
     */    
    void parse(unsigned char cc);

    /** serial object */
    Serial serial;
    int _available;     // is data available?
    float _latitude;    // temp storage, latitude
    float _longitude;   // temp storage, longitude
    float _hdop;        // horiz dilution of precision
    float _course_deg;  // course in degrees
    float _speed_mps;   // speed in m/s
    int _sat_count;     // satellite count
    //unsigned char buf[MAX_LENGTH]; // FIFO buffer for processing UBX
    //int in;             // buffer input, write to here
    //int out;            // buffer output, read from here   
};

#endif