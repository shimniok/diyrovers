/*
 * print.h
 *
 *  Created on: Jan 6, 2014
 *      Author: mes
 */

#ifndef MYPRINT_H_
#define MYPRINT_H_

#include <stdio.h>
#include <stdint.h>

#ifdef __CPLUSPLUS
extern "C" {
#endif

// from Arduino source
extern "C" size_t printNumber(FILE *f, unsigned long n);

// from Arduino source
extern "C" size_t printInt(FILE *f, long n);

// from Arduino source
extern "C" size_t printFloat(FILE *f, double number, uint8_t digits);

#ifdef __CPLUSPLUS
}
#endif

#endif /* MYPRINT_H_ */
