#ifndef __venus638flpx_h__
#define __venus638flpx_h__

#include "GPS.h"
#include "TinyGPS.h"

class Venus638flpx
{
public:
    /**
     * create a new interface for Venus638flpx
     */
    Venus638flpx(PinName tx, PinName rx);

    /**
     * Initalize everything necessary for the GPS to collect the required data
     */
    void init();

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
     * Configure GPS update rate
     */
    void setUpdateRate(int rate);

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

    int getAvailable(void);

private:
    /**
     * Configure which Nmea messages to enable/disable
     */
    void setNmeaMessages(char gga, char gsa, char gsv, char gll, char rmc, char vtg);

    /**
     * Serial interrupt handler
     */    
    void recv_handler(void);

    /** private serial object */
    Serial serial;

    /** private NMEA parser */
    TinyGPS nmea;
};

#endif