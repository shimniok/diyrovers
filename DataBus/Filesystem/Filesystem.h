/*
 * Filesystem.h
 *
 *  Created on: Dec 20, 2013
 *      Author: mes
 */

#ifndef FILESYSTEM_H_
#define FILESYSTEM_H_

#include "mbed.h"
#include "SDFileSystem.h"

class Filesystem {
public:
	Filesystem();
	void setcwd(const char *s);
	void getcwd();
private:
	SDFileSystem fs1;
	LocalFileSystem fs2;
	char cwd[128];
};


#endif /* FILESYSTEM_H_ */
