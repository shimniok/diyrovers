/*
 * Fixed.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: mes
 */

#include "Fixed.h"
extern "C" {
#include "libfixmath/fixmath.h"
}

Fixed::Fixed() {
	value = 0;
}

Fixed::Fixed(const double v) {
	value = fix16_from_dbl(v);
}

Fixed::Fixed(const float v) {
	value = fix16_from_float(v);
}

Fixed::Fixed(const int v) {
	value = fix16_from_int(v);
}

Fixed::~Fixed() {
	delete &value;
	delete this;
}

// Copy constructor
Fixed::Fixed(const Fixed &source) {
	value = source.value;
}

Fixed Fixed::operator+(const Fixed &b) {
	Fixed result;
	result.value = fix16_add(value, b.value);
	return result;
}

Fixed& Fixed::operator+=(const Fixed &b) {
	value = fix16_add(value, b.value);
	return *this;
}

Fixed Fixed::operator-(const Fixed &b) {
	Fixed result;
	result.value = fix16_sub(value, b.value);
	return result;
}

Fixed& Fixed::operator-=(const Fixed &b) {
	value = fix16_sub(value, b.value);
	return *this;
}

Fixed Fixed::operator*(const Fixed &b) {
	Fixed result;
	result.value = fix16_mul(value, b.value);
	return result;
}

Fixed Fixed::operator/(const Fixed &b) {
	Fixed result;
	result.value = fix16_div(value, b.value);
	return result;
}

Fixed operator/(const float a, const Fixed &b) {
	return Fixed(Fixed(a) / b);
}

bool Fixed::operator>=(const Fixed &b) {
	return (value >= b.value);
}

bool Fixed::operator<=(const Fixed &b) {
	return (value <= b.value);
}

bool Fixed::operator>(const Fixed &b) {
	return (value > b.value);
}

bool Fixed::operator<(const Fixed &b) {
	return (value < b.value);
}

Fixed Fixed::operator-() {
	Fixed result;
	value = -value;
	return result;
}

Fixed& Fixed::operator=(const Fixed &a) {
	value = a.value;
	return *this;
}

bool Fixed::operator==(const Fixed &b) {
	return (value == b.value);
}

bool Fixed::operator==(const float b) {
	return (value == fix16_from_float(b));
}

bool Fixed::operator!=(const Fixed &b) {
	return (value != b.value);
}

bool Fixed::operator!=(const float b) {
	return (value != fix16_from_float(b));
}

Fixed::operator int() {
	return this->toInt();
}

Fixed::operator float() {
	return this->toFloat();
}

Fixed::operator double() {
	return this->toDouble();
}

Fixed fabs(const Fixed &a) {
	Fixed result(a);
	if (result.value < 0) result = -result;
	return result;
}

int Fixed::toInt() {
	return fix16_to_int(value);
}

double Fixed::toDouble() {
	return fix16_to_dbl(value);
}

float Fixed::toFloat() {
	return fix16_to_float(value);
}

