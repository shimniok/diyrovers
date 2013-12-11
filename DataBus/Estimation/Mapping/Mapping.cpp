#include "mbed.h" // debugging
#include "Mapping.h"

/*
 * Provide a series of lat/lon coordinates/waypoints, and the object
 * automatically generates a bounding rectangle, and determines how to
 * translate between lat/lon and cartesian x/y
 */
void Mapping::init(int count, GeoPosition *p)
{
    double latMin = 360;
    double latMax = -360;
    double lonMin = 360;
    double lonMax = -360;
    
    // Find minimum and maximum lat/lon
    for (int i=0; i < count; i++, p++) {
        //fprintf(stdout, "%d (%.6f, %.6f)\n", i, p->latitude(), p->longitude());
        if (p->latitude() > latMax)
            latMax = p->latitude();
        if (p->latitude() < latMin)
            latMin = p->latitude();
            
        if (p->longitude() > lonMax)
            lonMax = p->longitude();
        if (p->longitude() < lonMin)
            lonMin = p->longitude();
    }

    // Set the arbitrary cartesian origin to southwest corner
    lonZero = lonMin;
    latZero = latMin;

    //fprintf(stdout, "min: (%.6f, %.6f)\n", latMin, lonMin);
    //fprintf(stdout, "max: (%.6f, %.6f)\n", latMax, lonMax);

    // Three positions required to scale
    GeoPosition sw(latMin, lonMin);
    GeoPosition nw(latMax, lonMin);
    GeoPosition se(latMin, lonMax);
    
    // Find difference between lat and lon min/max
    float dlat = (latMax - latMin);
    float dlon = (lonMax - lonMin);
    
    //fprintf(stdout, "dlat=%.6f dlon=%.6f\n", dlat, dlon);
    
    // Compute scale based on distances between edges of rectangle
    // and difference between min/max lat and min/max lon
    lonToX = sw.distanceTo(se) / dlon;
    latToY = sw.distanceTo(nw) / dlat;

    fprintf(stdout, "lonToX=%.10f\nlatToY=%.10f\n", lonToX, latToY);

    return;    
}

void Mapping::geoToCart(GeoPosition pos, CartPosition *cart)
{
    cart->set( lonToX * (pos.longitude() - lonZero),
               latToY * (pos.latitude() - latZero) );

    return;
}

void Mapping::cartToGeo(float x, float y, GeoPosition *pos)
{
    pos->set( (y / latToY) + latZero, 
              (x / lonToX) + lonZero);

    return; 
}

void Mapping::cartToGeo(CartPosition cart, GeoPosition *pos)
{
    cartToGeo(cart._x, cart._y, pos);
    
    return;
}

