/*
 * sys.cpp
 *
 * Various routines required by newlib
 *
 *  Created on: Dec 10, 2013
 *      Author: mes
 */
#include <sys/types.h>
#include <errno.h>

extern "C" {

	int _kill(int pid, int sig) {
		return -1;
	}

	void _exit(int status) {
		while(1);
	}

	int _getpid(void) {
		return 1;
	}

	// TODO: Figure out how to implement _sbrk later
	void *_sbrk(unsigned int incr) {
		return 0;
	}

}
