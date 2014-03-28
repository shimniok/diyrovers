/*
 * Channel.h
 *
 *  Created on: Mar 27, 2014
 *      Author: mes
 */

#ifndef CHANNEL_H_
#define CHANNEL_H_

#include "mbed.h"
#include <stddef.h>

/** maximum size of channel buffers */
const int _BUFSIZ=1024;

class Channel {
public:

	/** create a new channel for specified serial object */
	Channel(int channel);

	/** fill buffer, called by serial handler, channel demux */
	ssize_t fill(void *buf, size_t count);

	/** read from this channel's buffer */
	ssize_t read(void *buf, size_t count);

	/** write to this channel */
	ssize_t write(const void* buffer, size_t length);

private:
	int _channel;
	int _out;
	int _in;
	char _buf[_BUFSIZ];
};



#endif /* CHANNEL_H_ */
