/*
 * SystemState.c
 *
 *  Created on: Jan 1, 2014
 *      Author: mes
 */

#include "mbed.h"
#include <stdlib.h>
#include <string.h>
#include "SystemState.h"

volatile int inState = 0; // push pointer
volatile int outState = 0; // pull pointer
SystemState *state;
SystemState mystate[SSBUF];

// TODO 1 convert this to generic fifo, use for systemstate and gyro history

void state_clear( SystemState *s )
{
    s->millis = 0;
    s->current = s->voltage = 0.0;
    s->g[0] = s->g[1] = s->g[2] = 0;
    s->gyro[0] = s->gyro[1] = s->gyro[2] = 0;
    s->gTemp = 0;
    s->a[0] = s->a[1] = s->a[2] = 0;
    s->m[0] = s->m[1] = s->m[2] = 0;
    s->gHeading = s->cHeading = 0.0;
    s->gpsLatitude = s->gpsLongitude = s->gpsCourse_deg = s->gpsSpeed_mps = s->gpsHDOP = 0.0;
    s->lrEncDistance = s->rrEncDistance = 0.0;
    s->lrEncSpeed = s->rrEncSpeed = s->encHeading = 0.0;
    s->estHeading = s->estLatitude = s->estLongitude = 0.0;
    s->estX = s->estY = 0.0;
    s->nextWaypoint = 0;
    s->bearing = s->distance = 0.0;
}

bool fifo_init() {
    // Allocate memory for system state buffer
    // We're doing this to (hopefully) save some flash size
    //state = (SystemState *) malloc(SSBUF*sizeof(SystemState));
	state = mystate;
	fifo_reset();
	return (state != NULL);
}

void fifo_reset() {
	// initialize in/out pointers
	__disable_irq();
	inState = outState = 0;
	__enable_irq();
}

bool fifo_available() {
	return (inState != outState);
}

bool fifo_push(SystemState *s) {
	__disable_irq();
	inState++;                      // Get next state struct in the buffer
    inState &= (SSBUF-1);           // Wrap around
    __enable_irq();
	memcpy((void *) &state[inState], (void *) s, sizeof(SystemState));

	return (inState != outState);
}

SystemState *fifo_pull() {
	SystemState *s = NULL;

	if (fifo_available()) {
		__disable_irq();
		outState++;                     // increment
		outState &= (SSBUF-1);          // wrap
		s = fifo_last();
		__enable_irq();
	}

	return s;
}

SystemState *fifo_first() {
	return &state[inState];
}

SystemState *fifo_last() {
	return &state[outState];
}

