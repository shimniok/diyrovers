/*
 * libcr_helpers.c
 *
 *  Created on: Dec 10, 2013
 *      Author: mes
 */

#include <sys/types.h>

// TODO: figure out what to put in these functions

void _abort() {}

void _exit(int status) {
	while (1);
}

caddr_t _sbrk(int incr) {
	return (caddr_t) 0;
}

void _kill() {}

void _getpid() {}

void _fstat() {}

void _close() {}

void _lseek() {}

void _read() {}

void _write() {}

void _isatty() {}

