#ifndef __GLOBALS_H
#define __GLOBALS_H

/** Global parameters */

#define PI 3.1415926535897932

// Axis parameters
#define _x_ 0
#define _y_ 1
#define _z_ 2

// Waypoint queue parameters
#define MAXWPT    10
#define ENDWPT    9999.0

#include "Steering.h"
#include "Serial.h"
#include "SerialGraphicLCD.h"
#include "Buttons.h"
#include "L3G4200D.h"

extern Steering steering;
extern Serial pc;
extern SerialGraphicLCD lcd;
extern Buttons keypad;

#endif
