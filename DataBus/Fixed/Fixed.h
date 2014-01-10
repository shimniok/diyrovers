/*
 * Fixed.h
 *
 *  Created on: Jan 9, 2014
 *      Author: mes
 */

#ifndef FIXED_H_
#define FIXED_H_

#include <iostream>
#include <ostream>
#include <fstream>
extern "C" {
#include "libfixmath/fix16.h"
}

class Fixed
{
public:
	static const int ovf=0x02;
	static const int nan=0x03;
	static const int inf=0x04;

	Fixed();
	Fixed(const double v);
    Fixed(const float v);
    Fixed(const int v);
    Fixed(const Fixed &source);
    ~Fixed();

    Fixed  operator+(const Fixed &b);
	Fixed& operator+=(const Fixed &b);

	Fixed  operator-(const Fixed &b);
	Fixed& operator-=(const Fixed &b);
	Fixed  operator-();

	Fixed  operator*(const Fixed &b);
	Fixed& operator*=(const Fixed &b);

	Fixed  operator/(const Fixed &b);
	Fixed& operator/=(const Fixed &b);
    friend Fixed operator/(const float a, const Fixed &b);

    bool operator>=(const Fixed &b);
    bool operator<=(const Fixed &b);
    bool operator>(const Fixed &b);
    bool operator<(const Fixed &b);


    Fixed& operator=(const Fixed &a);

    bool operator==(const Fixed &b);
    bool operator==(const float b);
    bool operator!=(const Fixed &b);
    bool operator!=(const float b);

    operator int();
    operator float();
    operator double();

    friend Fixed fabs(const Fixed &a);

    int toInt();
    double toDouble();
    float toFloat();

private:
    fix16_t value;
};



#endif /* FIXED_H_ */
