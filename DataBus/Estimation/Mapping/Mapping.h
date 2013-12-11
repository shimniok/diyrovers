#ifndef __MAPPING_H
#define __MAPPING_H

#include "GeoPosition.h"
#include "CartPosition.h"

/** Maps GeoPosition latitude/longitude to CartPosition cartesian x,y
 */
class Mapping {
public:
    /** Initialize mapping with a number of GeoPosition coordinates
     *  The maximum/minimum values for lat/lon are then used to form
     *  four coordinates and linear interpolation is used to map lat/lon to y/x
     */
    void init(int count, GeoPosition *p);
    /** Convert a GeoPosition to a CartPosition
     * @param pos is the lat/lon pair
     * @returns cart is the converted cartesian coordinate pair
     */
    void geoToCart(GeoPosition pos, CartPosition *cart);
    /** Convert a GeoPosition to a CartPosition
     * @param x is the cartesian x coordinate
     * @param y is the cartesian y coordinate
     * @returns pos is the converted GeoPosition lat/lon coordinate pair
     */
    void cartToGeo(float x, float y, GeoPosition *pos);
    /** Convert a GeoPosition to a CartPosition
     * @param cart is the x,y cartesian coordinate pair
     * @returns pos is the converted GeoPosition lat/lon coordinate pair
     */
    void cartToGeo(CartPosition cart, GeoPosition *pos);

private:
    double lonToX;
    double latToY;
    double latZero;
    double lonZero;
};
#endif
