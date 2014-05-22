#include "mbed.h"
#include "Steering.h"
#include "math.h"
#include "util.h"

Steering::Steering(PinName pin):
	_steering(pin)
{
}


void Steering::initSteering()
{
    if (Config::loaded) {
        // Setup steering servo
        _steering = Config::steerZero;
    } else {
        _steering = 0.4;
    }
    // TODO: 3 parameterize this in config file
    _steering.calibrate(0.005, 45.0);
}


void Steering::setSteering(float steerAngle)
{
    // Convert steerAngle to servo value
    // Testing determined near linear conversion between servo ms setting and steering angle
    // up to 20*.  Assumes a particular servo library with range = 0.005
    // In that case, f(SA) = servoPosition = 0.500 + SA/762.5
    // between 20 and 24* the slope is approximately 475
    // What if we ignore the linearity and just set to a max angle
    // also range is 0.535-0.460 --> slope = 800
    // steering = 0.500 + (double) steerAngle / 762.5;
    //
    _steering = 0.500 - (double) steerAngle / 808.0; // TODO 0: parameterize through config
}


float Steering::operator =(float steerAngle)
{
	setSteering(steerAngle);
    return steerAngle;
}


/** Calculate a steering angle based on relative bearing
 *
 */
float Steering::calcSA(float theta)
{
    return calcSA(theta, -1.0); // call with no limit
}

/** calcSA
 * minRadius -- radius limit (minRadius < 0 disables limiting)
 */
float Steering::calcSA(float theta, float minRadius)
{
    float radius;
    float SA;
    bool neg = (theta < 0);

    // I haven't had time to work out why the equation is slightly offset such
    // that negative angle produces slightly less steering angle
    //
    if (neg) theta = -theta;

    // The equation peaks out at 90* so clamp theta artifically to 90, so that
    // if theta is actually > 90, we select max steering
    if (theta > 90.0) theta = 90.0;

    // Compute |radius| based on intercept distance and specified angle with extra gain to
    // overcome steering slop, misalignment, sidehills, etc.
    radius = Config::intercept / ( 2 * sin(toRadians(theta)) );

    if (minRadius > 0) {
        if (radius < minRadius) radius = minRadius;
    }

    // Now calculate steering angle based on wheelbase and track width
    SA = toDegrees(asin(Config::wheelbase / (radius - Config::track/2)));
    // The above ignores the effect of speed on required steering angle.
    // Even when under the limits of traction, understeer means more angle
    // is required to achieve a turn at higher speeds than lower speeds.
    // To consider this, we'd need to measure the understeer gradient of
    // the vehicle (thanks to Project240 for this insight) and include
    // that in the calculation.

    if (neg) SA = -SA;

    return SA;
}

/**
 * Bxy - robot coordinates
 * Axy - previous waypoint coords
 * Cxy - next waypoint coords
 */
float Steering::crossTrack(float Bx, float By, float Ax, float Ay, float Cx, float Cy)
{
/*
    // Compute rise for prev wpt to bot; or compute vector offset by A(x,y)
    float Rx = (Bx - Ax);
    // compute run for prev wpt to bot; or compute vector offset by A(x,y)
    float Ry = (By - Ay);
    // dx is the run for the path
    float dx = Cx - Ax;
    // dy is the rise for the path
    float dy = Cy - Ay;
    // this is hypoteneuse length squared
    float ACd2 = dx*dx+dy*dy;
    // length of hyptoenuse
    float ACd = sqrtf( ACd2 );

    float Rd = Rx*dx + Ry*dy;
    float t = Rd / ACd2;
    // nearest point on current segment
    float Nx = Ax + dx*t;
    float Ny = Ay + dy*t;
    // Cross track error
    float NBx = Nx-Bx;
    float NBy = Ny-By;
    float cte = sqrtf(NBx*NBx + NBy*NBy);
    return cte;
*/
    return 0;
}

//static int skip=0;

float Steering::pathPursuitSA(float hdg, float Bx, float By, float Ax, float Ay, float Cx, float Cy)
{
    // Leg vector
    float Lx = Cx - Ax;
    float Ly = Cy - Ay;
    // Robot vector
    float Rx = Bx - Ax;
    float Ry = By - Ay;
    float sign = 1;

    // Find the goal point, a projection of the bot vector onto the current leg, moved ahead
    // along the path by the lookahead distance
    float legLength = sqrtf(Lx*Lx + Ly*Ly); // ||L||
    float proj = (Lx*Rx + Ly*Ry)/legLength; // R dot L/||L||, projection magnitude, bot vector onto leg vector
    float LAx = (proj + Config::intercept)*Lx/legLength; // find projection point + lookahead, along leg, relative to Bx
    float LAy = (proj + Config::intercept)*Ly/legLength;
    // Compute a circle that is tangential to bot heading and intercepts bot
    // and goal point (LAx,LAy), the intercept circle. Then compute the steering
    // angle to trace that circle. (x,y because 0 deg points up not right)
    float brg = clamp360( toDegrees(atan2(LAx-Rx,LAy-Ry)) );
    //if (brg >= 360.0) brg -= 360.0;
    //if (brg < 0) brg += 360.0;
    // would be nice to add in some noise to heading info
    float relBrg = clamp180(brg - hdg);
    // The steering angle equation actually peaks at relBrg == 90 so just clamp to this
    if (relBrg > 89.5) {
        relBrg = 89.5;
    } else if (relBrg < -89.5) {
        relBrg = -89.5;
    }
    // Compute radius based on intercept distance and specified angle
    // The sin equation will produce a negative radius, which causes problems
    // when subtracting track/2.0, so just take absolute value and multiply sign
    // later on
    sign = (relBrg < 0) ? -1 : 1;
    float radius = Config::intercept/fabs(2*sin(toRadians(relBrg)));
    // optionally, limit radius min/max
    // Now compute the steering angle to achieve the circle of 
    // Steering angle is based on wheelbase and track width
    return ( sign * toDegrees(asin(Config::wheelbase / (radius - Config::track/2.0))) );
}


float Steering::purePursuitSA(float hdg, float Bx, float By, float Ax, float Ay, float Cx, float Cy)
{
    float SA;

    // Compute rise for prev wpt to bot; or compute vector offset by A(x,y)
    float Rx = (Bx - Ax);
    // compute run for prev wpt to bot; or compute vector offset by A(x,y)
    float Ry = (By - Ay);
    // dx is the run for the path
    float dx = Cx - Ax;
    // dy is the rise for the path
    float dy = Cy - Ay;
    // this is hypoteneuse length squared
    float ACd2 = dx*dx+dy*dy;
    // length of hyptoenuse
    float ACd = sqrtf( ACd2 );

    float Rd = Rx*dx + Ry*dy;
    float t = Rd / ACd2;
    // nearest point on current segment
    float Nx = Ax + dx*t;
    float Ny = Ay + dy*t;
    // Cross track error
    float NBx = Nx-Bx;
    float NBy = Ny-By;
    float cte = sqrtf(NBx*NBx + NBy*NBy);
    float NGd;

    float myLookAhead;

    if (cte <= Config::intercept) {
        myLookAhead = Config::intercept;
    } else {
        myLookAhead = Config::intercept + cte;
    }

    NGd = sqrt( myLookAhead*myLookAhead - cte*cte );
    float Gx = NGd * dx/ACd + Nx;
    float Gy = NGd * dy/ACd + Ny;

    float hdgr = hdg*PI/180;

    float BGx = (Gx-Bx)*cos(hdgr) - (Gy-By)*sin(hdgr);
    float c = (2 * BGx) / (myLookAhead*myLookAhead);

    float radius;

    if (c != 0) {
        radius = 1/c;
    } else {
        radius = 999999.0;
    }

    // Now calculate steering angle based on wheelbase and track width
    SA = toDegrees(asin(Config::wheelbase / (radius - Config::track/2)));

    return SA;
}
