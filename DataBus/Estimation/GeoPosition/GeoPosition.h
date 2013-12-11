#ifndef __GEOPOSITION_H
#define __GEOPOSITION_H

#ifndef _PI
#define _PI 3.141592653
#endif

#define degrees(x) ((x)*180/_PI)
#define radians(x) ((x)*_PI/180)

/** Geographical position and calculation. Most of this comes from http://www.movable-type.co.uk/scripts/latlong.html
 *
 */
class GeoPosition {
public:

    /** Create a new emtpy position object
     *
     */
    GeoPosition();

    /** Create a new position with the specified latitude and longitude. See set()
     *
     *  @param latitude is the latitude to set
     *  @param longitude is the longitude to set
     */
    GeoPosition(double latitude, double longitude);
    
    /** Get the position's latitude
     *
     *  @returns the position's latitude
     */
    double latitude();
    
    /** Get the position's longitude
     *
     *  @returns the position's longitude
     */
    double longitude();
    
    /** Set the position's location to another position's coordinates
     *
     *  @param pos is another position from which coordinates will be copied
     */
    void set(GeoPosition pos);
    
    /** Set the position's location to the specified coordinates
     *
     *  @param latitude is the new latitude to set
     *  @param longitude is the new longitude to set
     */
    void set(double latitude, double longitude);
    
    /** Move the location of the position by the specified distance and in
     *  the specified direction
     *
     *  @param course is the direction of movement in degrees, absolute not relative
     *  @param distance is the distance of movement along the specified course in meters
     */
    void move(float course, float distance);

    /** Get the bearing from the specified origin position to this position.  To get
     *  relative bearing, subtract the result from your heading.
     *
     *  @param from is the position from which to calculate bearing
     *  @returns the bearing in degrees
     */
    float bearing(GeoPosition from);

    float bearingFrom(GeoPosition from);

    float bearingTo(GeoPosition to);
    
    /** Get the distance from the specified origin position to this position
     *
     *  @param from is the position from which to calculate distance
     *  @returns the distance in meters
     */
    float distance(GeoPosition from);
    
    float distanceTo(GeoPosition to);
    
    float distanceFrom(GeoPosition from);
    
    /** Set an arbitrary timestamp on the position
     *
     * @param timestamp is an integer timestamp, eg., seconds, milliseconds, or whatever
     */
     void setTimestamp(int time);

    /** Return a previously set timestamp on the position
     *
     * @returns the timestamp (e.g., seconds, milliseconds, etc.)
     */     
     int getTimestamp(void);

private:
    double _R;          /** Earth's mean radius */
    double _latitude;   /** The position's latitude */
    double _longitude;  /** The position's longitude */
    double _northing;   /** The position's UTM northing coordinate */
    double _easting;    /** The position's UTM easting coordinate */
    int _time;          /** Timestamp */
};

#endif