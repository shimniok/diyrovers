#include "mbed.h" // debug
#include "CartPosition.h"

#include "globals.h"

CartPosition::CartPosition(void)
{
    set(0,0);
}

CartPosition::CartPosition(float x, float y)
{
    set(x,y);
}

// TODO: 3 fix _x, _y to be private with getters setters

void CartPosition::set(CartPosition p)
{
    x = p.x;
    y = p.y;
}

void CartPosition::set(float newx, float newy)
{
    x = newx;
    y = newy;
}


float CartPosition::bearingTo(CartPosition to)
{
    // x and y aren't backwards; it's to correct for the differences between
    // geometry and navigation. In the former, angles are measured from the x axis,
    // in the latter, from the y axis.
    return 180/PI * atan2(to.x-x, to.y-y); 
}


float CartPosition::distanceTo(CartPosition to)
{
    float dx = to.x-x;
    float dy = to.y-y;
    
    return sqrt( dx*dx + dy*dy );
}

void CartPosition::move(float bearing, float distance)
{
    // x and y aren't backwards; it's to correct for the differences between
    // geometry and navigation. In the former, angles are measured from the x axis,
    // in the latter, from the y axis.
    float r = bearing * PI / 180;
    x += distance * sin( r );
    y += distance * cos( r );
    
    return;
}
