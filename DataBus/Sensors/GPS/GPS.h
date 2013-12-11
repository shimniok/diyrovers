// For SiRF III

#ifndef __GPS_H
#define __GPS_H

/**
 * GPS "interface" class
 */

#include "mbed.h"

class GPS {
public:
    /**
     * create a new instance of GPS
     */
    //GPS(PinName tx, PinName rx); // serial only

    /**
     * Initalize everything necessary for the GPS to collect the required data
     */
    virtual void init(void) = 0;

    /**
     * Return serial object
     */
    virtual Serial *getSerial(void) = 0;

    /**
     * Set baud rate
     */
    virtual void setBaud(int baud) = 0;

    /**
     * Disable serial data collection
     */
    virtual void disable(void) = 0;
     
    /**
     * Enable serial data collection
     */
    virtual void enable(void) = 0;

    /**
     * Enable verbose messages for debugging
     */
    virtual void enableVerbose(void) = 0;
    
    /**
     * Disable verbose messages for debugging
     */
    virtual void disableVerbose(void) = 0;
   
    /**
     * get latitude
     */
    virtual double latitude(void) = 0;
         
    /**
     * get longitude
     */
    virtual double longitude(void) = 0;
     
    /**
     * Get Horizontal Dilution of Precision
     * @return float horizontal dilution of precision
     */
    virtual float hdop(void) = 0;
    
    /**
     * get count of active satellites
     */
    virtual int sat_count(void) = 0;
    
    /**
     * get speed in m/s
     */
    virtual float speed_mps(void) = 0;
    
    /**
     * get heading in degrees
     */
    virtual float heading_deg(void) = 0;
    
    /**
     * determine if data is available to be used
     */
    virtual bool available(void) = 0;
    
    /**
     * reset the data available flag
     */
    virtual void reset_available(void) = 0;
};

#endif