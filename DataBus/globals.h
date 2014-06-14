#ifndef __GLOBALS_H
#define __GLOBALS_H

/** Global parameters */

#define PI 3.1415926535897932

// Waypoint queue parameters
#define MAXWPT    10
#define ENDWPT    9999.0

#include "Steering.h"
#include "Buttons.h"
#include "Serial.h"
#include "Display.h"

extern Display display;
extern Steering steering;
extern Serial pc;
extern Buttons keypad;

#endif
