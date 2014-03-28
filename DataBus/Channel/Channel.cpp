/*
 * Channel.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: mes
 */

#include "Channel.h"

#if 0

Channel::Channel(int channel):
	_channel(channel)
{
	// nothing to do really
}



ssize_t Channel::fill(void *buf, size_t count) {
	ssize_t c=0;
	while (c < count && c < _BUFSIZ && _out != _in) {
		_buf[_out++] = buf[c++];
		_out &= (_BUFSIZ-1); // wrap buffer
	}
	return c;
}

ssize_t Channel::read(void *buf, size_t count) {
	ssize_t c=0;
	while (c < count && c < _BUFSIZ && _out != _in) {
		buf[c++] = _buf[_out++];
		_out &= (_BUFSIZ-1); // wrap buffer
	}
	return c;
}

ssize_t Channel::write(const void* buffer, size_t length) {
	ssize_t result = 0;
	////////////// mutex for channel write
	// write channel
	result = _serial->write(&_channel, 1);
	// write length
	result += _serial->write(&length, sizeof(size_t));
	//
	if (result == (1+sizeof(size_t))) {
		// write buffer
		result = _serial->write(buffer, length);
	} else {
		result = -1;
	}
	////////////// mutex for channel write
	return result;
}
#endif

