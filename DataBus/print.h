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

/** print an unsigned long number to FILE
 *
 * @param f is the FILE * to which the data will be sent
 * @param n is the number to print
 * @return the number of bytes printed
 */
extern "C" size_t printNumber(FILE *f, unsigned long n);

/** print a signed long number to FILE
 *
 * @param f is the FILE * to which the data will be sent
 * @param n is the number to print
 * @return the number of bytes printed
 */
extern "C" size_t printInt(FILE *f, long n);

/** print a double to FILE
 *
 * @param f is the FILE * to which the data will be sent
 * @param number is the number to print
 * @param digits is the number of digits to print after the decimal point
 * @return the number of bytes printed
 */
extern "C" size_t printFloat(FILE *f, double number, uint8_t digits);

/** print an int as hex to FILE
 *
 * @param f is the FILE * to which the data will be sent
 * @param n is the number to print
 * @param digits is the number of hex digits to print
 * @return the number of bytes printed
 */
extern "C" size_t printHex(FILE *f, long n, uint8_t digits);

#ifdef __CPLUSPLUS
}
#endif

#endif /* MYPRINT_H_ */
