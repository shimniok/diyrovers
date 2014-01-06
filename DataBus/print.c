/*
 * print.c
 *
 *  Created on: Jan 6, 2014
 *      Author: mes
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>

// from Arduino source
size_t printNumber(FILE *f, unsigned long n)
{
    char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';

    do {
        unsigned long m = n;
        n /= 10;
        char c = m - 10 * n;
        *--str = c + '0';
    } while(n);

    return fputs(str, f);
}

// from Arduino source
size_t printInt(FILE *f, long n)
{
    int t = 0;
    if (n < 0) {
        t = fputc('-', f);
        n = -n;
    }
    return printNumber(f, n) + t;
}

// from Arduino source
size_t printFloat(FILE *f, double number, uint8_t digits)
{
    size_t n=0;

    if (isnan(number)) return fputs("nan", f);
    if (isinf(number)) return fputs("inf", f);
    if (number > 4294967040.0) return fputs("ovf", f);  // constant determined empirically
    if (number <-4294967040.0) return fputs("ovf", f);  // constant determined empirically

    // Handle negative numbers
    if (number < 0.0) {
        n += fputc('-', f);
        number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i=0; i < digits; ++i)
        rounding /= 10.0;

    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    n += printInt(f, int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        n += fputc('.', f);
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        int toPrint = (int) remainder;
        n += fputc(toPrint+'0', f);
        remainder -= toPrint;
    }

    return n;
}
