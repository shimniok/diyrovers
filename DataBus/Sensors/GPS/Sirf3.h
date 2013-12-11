#ifndef __SIRF3_H__
#define __SIRF3_H__

#include "mbed.h"
#include "GPS.h"
#include "TinyGPS.h"

class Sirf3: public GPS {
public:
    /**
     * create a new SiRF III interface
     */
    Sirf3(PinName tx, PinName rx);

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
     * stub function for compatibility
     */
    int getAvailable(void);

    /**
     * stub function for compatibility
     * Configure GPS update rate (doesn't do anything on Sirf3)
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

private:
    void recv_handler(void);
    void ggaMessage(bool enable);
    void gsaMessage(bool enable);
    void gsvMessage(bool enable);
    void gllMessage(bool enable);
    void rmcMessage(bool enable);
    void vtgMessage(bool enable);
    Serial serial;
    TinyGPS nmea;
};

#endif