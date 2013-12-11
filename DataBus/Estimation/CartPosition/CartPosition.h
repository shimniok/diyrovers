#ifndef __CARTPOSITION_H
#define __CARTPOSITION_H

/** Geographical position and calculation based on cartesian coordinates
 */
class CartPosition {
public:
    /** Create a new cartesian coordinate object
     */
    CartPosition(void);
    /** Create a new cartesian coordinate object
     * @param x sets x coordinate
     * @param y sets y coordinate 
     */
    CartPosition(float x, float y);
    /** Sets coordinates for object
     * @param x sets x coordinate
     * @param y sets y coordinate 
     */
    void set(float x, float y);
    /** Sets coordinates for object
     * @param p sets coordinates of this object to that of p
     */
    void set(CartPosition p);
    /** Computes bearing to a position from this position
     * @param to is the coordinate to which we're calculating bearing
     */
    float bearingTo(CartPosition to);
    /** Computes distance to a position from this position
     * @param to is the coordinate to which we're calculating distance
     */
    float distanceTo(CartPosition to);
    /** Computes the new coordinates for this object given a bearing and distance
     * @param bearing is the direction traveled
     * @distance is the distance traveled
     */
    void move(float bearing, float distance);
    /** x coordinate of this object */
    float _x;
    /** y coordinate of this object */
    float _y;
};
#endif