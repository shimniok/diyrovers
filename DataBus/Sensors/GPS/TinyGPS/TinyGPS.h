/*
  TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
  Copyright (C) 2008-9 Mikal Hart
  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Ported to mbed by Michael Shimniok
*/

#include "mbed.h"
#include "types.h"

#ifndef TinyGPS_h
#define TinyGPS_h

#define _GPS_VERSION 9 // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001
//#define _GPS_NO_STATS

/** TinyGPS - a small GPS library for Arduino providing basic NMEA parsing Copyright (C) 2008-9 Mikal Hart
 * All rights reserved. Modified by Michael Shimniok 
 */

class TinyGPS
{
  public:
  
    /** Create a new GPS parsing object for parsing NMEA sentences
     */
    TinyGPS();
    
    /** Parse a single character received from GPS
     *
     * @param c is the character received from the GPS
     * @returns true if processing ok
     */
    bool encode(char c);
    
    /** Shorthand operator for encode()
     */
    TinyGPS &operator << (char c) {encode(c); return *this;}
    
    /** Return lat/long in hundred thousandths of a degree and age of fix in milliseconds
     * @returns latitude is the latitude of the most recent fix that was parsed
     * @returns longitude is the longitude of the most recent fix that was parsed
     * @returns fix_age is the age of the fix (if available from the NMEA sentences being parsed)
     */
    inline void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0)
    {
      if (latitude) *latitude = _latitude;
      if (longitude) *longitude = _longitude;
      if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? 
        GPS_INVALID_AGE : millis() - _last_position_fix;
    }

    /** Return the date and time from the parsed NMEA sentences
     *
     * @returns date as an integer value
     * @returns time as an integer value
     * @returns fix_age in milliseconds if available
     */
    inline void get_datetime(unsigned long *date, unsigned long *time, unsigned long *fix_age = 0)
    {
      if (date) *date = _date;
      if (time) *time = _time;
      if (fix_age) *fix_age = _last_time_fix == GPS_INVALID_FIX_TIME ? 
        GPS_INVALID_AGE : millis() - _last_time_fix;
    }

    /** signed altitude in centimeters (from GPGGA sentence)
     * @returns altitude in centimeters, integer
     */
    inline long altitude() { return _altitude; }

    /** course in last full GPRMC sentence in 100th of a degree
     * @returns course as an integer, 100ths of a degree
     */
    inline unsigned long course() { return _course; }
    
    /** speed in last full GPRMC sentence in 100ths of a knot
     * @returns speed in 100ths of a knot
     */
    unsigned long speed() { return _speed; }

    /* horizontal dilution of position in last full GPGGA sentence in 100ths
     * @returns hdop in 100ths
     */
    unsigned long hdop() { return _hdop; }

    /** number of satellites tracked in last full GPGGA sentence
     * @returns number of satellites tracked 
     */
    unsigned long sat_count() { return _sat_count; }

#ifndef _GPS_NO_STATS
    void stats(unsigned long *chars, unsigned short *good_sentences, unsigned short *failed_cs);
#endif

    /** returns position as double precision
     *
     * @returns latitude as double precision
     * @returns longitude as double precision
     * @returns fix age in milliseconds if available
     */
    inline void f_get_position(double *latitude, double *longitude, unsigned long *fix_age = 0)
    {
      long lat, lon;
      get_position(&lat, &lon, fix_age);
      // mes 04/27/12 increased fractional precision to 7 digits, was 5
      *latitude = lat / 10000000.0;
      *longitude = lon / 10000000.0;
    }

    /** Convert date and time of last parsed sentence to integers
     *
     * @returns year
     * @returns month
     * @returns day of month
     * @returns hour
     * @returns minute
     * @returns second
     * @returns hundreths
     * @returns fix_age in milliseconds if available
     */
    inline void crack_datetime(int *year, byte *month, byte *day, 
      byte *hour, byte *minute, byte *second, byte *hundredths = 0, unsigned long *fix_age = 0)
    {
      unsigned long date, time;
      get_datetime(&date, &time, fix_age);
      if (year) 
      {
        *year = date % 100;
        *year += *year > 80 ? 1900 : 2000;
      }
      if (month) *month = (date / 100) % 100;
      if (day) *day = date / 10000;
      if (hour) *hour = time / 1000000;
      if (minute) *minute = (time / 10000) % 100;
      if (second) *second = (time / 100) % 100;
      if (hundredths) *hundredths = time % 100;
    }

    /** returns altitude as a float
    */
    inline double f_altitude()    { return altitude() / 100.0; }
    
    /** returns course as a float
    */
    inline double f_course()      { return course() / 100.0; }
    
    /** returns speed in knots as a float
    */
    inline double f_speed_knots() { return speed() / 100.0; }
    
    /** returns speed in mph as a float 
    */
    inline double f_speed_mph()   { return _GPS_MPH_PER_KNOT * f_speed_knots(); }
    
    /** returns speed in meters per second as a float
    */
    inline double f_speed_mps()   { return _GPS_MPS_PER_KNOT * f_speed_knots(); }
    
    /** returns speed in km per hour as a float
    */
    inline double f_speed_kmph()  { return _GPS_KMPH_PER_KNOT * f_speed_knots(); }
    
    /** returns hdop as a float
    */
    inline double f_hdop()      { return hdop() / 100.0; }

    /** @returns library version
    */
    static int library_version() { return _GPS_VERSION; }

    /** determine if all sentences parsed
     *
     */
    inline bool ready() { return (_rmc_ready && _gga_ready); }

    /** determine if GSV sentence parsed since last reset_ready()
     */
    inline bool gsv_ready() { return _gsv_ready; }

    inline bool gga_ready() { return _gga_ready; }
    
    inline bool rmc_ready() { return _rmc_ready; }
   
    /** Reset the ready flags for all the parsed sentences
     */
    inline void reset_ready() { _gsv_ready = _rmc_ready = _gga_ready = false; }

    enum {GPS_INVALID_AGE = 0xFFFFFFFF, GPS_INVALID_ANGLE = 999999999, GPS_INVALID_ALTITUDE = 999999999, GPS_INVALID_DATE = 0,
      GPS_INVALID_TIME = 0xFFFFFFFF, GPS_INVALID_SPEED = 999999999, GPS_INVALID_FIX_TIME = 0xFFFFFFFF};

private:
    enum {_GPS_SENTENCE_GPGGA, _GPS_SENTENCE_GPRMC, _GPS_SENTENCE_GPGSV, _GPS_SENTENCE_OTHER};
    
    // properties
    unsigned long _time, _new_time;
    unsigned long _date, _new_date;
    long _latitude, _new_latitude;
    long _longitude, _new_longitude;
    long _altitude, _new_altitude;
    unsigned long  _speed, _new_speed;
    unsigned long  _course, _new_course;
    unsigned long  _hdop, _new_hdop;
    unsigned int _sat_count, _new_sat_count;
    unsigned long _last_time_fix, _new_time_fix;
    unsigned long _last_position_fix, _new_position_fix;

    // parsing state variables
    byte _parity;
    bool _is_checksum_term;
    char _term[15];
    byte _sentence_type;
    byte _term_number;
    uint8_t  _term_offset;
    bool _gps_data_good;
    bool _rmc_ready;
    bool _gga_ready;
    bool _gsv_ready;

#ifndef _GPS_NO_STATS
    // statistics
    unsigned long _encoded_characters;
    unsigned short _good_sentences;
    unsigned short _failed_checksum;
    unsigned short _passed_checksum;
#endif

    // internal utilities
    int from_hex(char a);
    int parse_int();
    unsigned long parse_decimal();
    unsigned long parse_degrees();
    bool term_complete();
    bool gpsisdigit(char c) { return c >= '0' && c <= '9'; }
    long gpsatol(const char *str);
    int gpsstrcmp(const char *str1, const char *str2);
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef double
#undef abs
#undef round 

#endif
