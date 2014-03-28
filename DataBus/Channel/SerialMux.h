/*
 * SerialMux.h
 *
 *  Created on: Mar 27, 2014
 *      Author: mes
 */

#ifndef SERIALMUX_H_
#define SERIALMUX_H_

#include "mbed.h"

const int MaxChannel=4;

class SerialMux {
public:

	/** Create a new SerialMux interface for a Serial object */
	SerialMux(Serial *s);

	/** Open (add) a new channel */
	int open();

	/** write to this channel */
	int puts(int channel, const char *s);

	/** Handler for serial object reads */
	void handler(void);

private:
	Serial *_serial;
};



#endif /* SERIALMUX_H_ */
