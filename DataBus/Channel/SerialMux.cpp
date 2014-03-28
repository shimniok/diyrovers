/*
 * SerialMux.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: mes
 */

#include "mbed.h"
#include "SerialMux.h"
#include "print.h"

SerialMux::SerialMux(Serial *s)
{
	_serial = s;
}

int SerialMux::open() {
	int result = -1;
	return result;
}


/** write to this channel */
int SerialMux::puts(int channel, const char *s) {
	ssize_t result = 0;
	char buf[8];

	// write channel
	if (s && channel >= 0 && channel < 4) {
		//memset(&buf, 0, 8); // guarantee zero termination

		// TODO 2 this really needs to deal with endian-ness, somehow
		int l = strlen(s);

		// TODO 2 what if s is too long?

		//memcpy(&buf[1], &l, sizeof(int)); // serialize length
		buf[0] = '0' + channel;
		buf[1] = (l>>24)         + '0';
		buf[2] = ((l>>16)& 0x0f) + '0';
		buf[3] = ((l>>8) & 0x0f) + '0';
		buf[4] = (l & 0x0f)      + '0';
		buf[5] = 0;

		////////////// mutex for channel write
		// write channel & length
		result = _serial->puts(buf);
		//
		//if (result > 0) {
			// write buffer
			result = _serial->puts(s);
		//} else {
			//result = -1;
		//}
		////////////// mutex for channel write
	} else {
		result = -1;
	}

	return result;
}

/** Handler for serial object reads */
void SerialMux::handler(void) {
	if (_serial->readable()) {
		// if counter is ?? we're expecting channel byte
		// read header byte, counter++
		// if counter is ?? we're expecting length bytes
		// if counter is < length, we're expecting data bytes
		// read a byte and stick it in the appropriate buffer
		// problem is that readHandler() has to know about all the channel objects.
	}
}
