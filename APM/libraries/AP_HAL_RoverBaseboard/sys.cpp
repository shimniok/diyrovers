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

int _isatty(int fh) {
    return -1;
}

ssize_t _write(const void* buffer, size_t length) {
	return 0;
}

ssize_t _read(void* buffer, size_t length) {
	return 0;
}

int _close() {
	return -1;
}

off_t _lseek(off_t offset, int whence) {
	return 0;
}

int _fstat(int fh) {
	return -1;
}

}
