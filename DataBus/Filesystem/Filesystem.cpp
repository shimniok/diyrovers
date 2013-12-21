/*
 * Filesystem.cpp
 *
 * Initialize filesystems
 *
 *  Created on: Dec 20, 2013
 *      Author: mes
 */

#include "Filesystem.h"

Filesystem::Filesystem():
	fs1(p5, p6, p7, p8, "log")			// mosi, miso, sclk, cs
, 	fs2("etc")							// mbed local filesystem
{
	// nothing to do, really. I suppose we could make directories... I dunno.
}

void Filesystem::setcwd(const char *s) {
}

void Filesystem::getcwd() {
}

