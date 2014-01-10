/*
 * FixedMatrix.h
 *
 *  Created on: Jan 9, 2014
 *      Author: mes
 */

#ifndef FIXEDMATRIX_H_
#define FIXEDMATRIX_H_

#include "assert.h"
#include "Fixed.h"
#include <vector>

class FixedMatrix {
public:

	FixedMatrix(const FixedMatrix &nfm);
	FixedMatrix(const int mm, const int nn);
	~FixedMatrix();

	// assignment
	FixedMatrix& operator=(const FixedMatrix& b);

	// addition
	FixedMatrix operator+(const FixedMatrix& b);
	FixedMatrix operator+(const Fixed& b);
    friend FixedMatrix operator+(FixedMatrix const&, FixedMatrix const&);
	FixedMatrix& operator+=(const FixedMatrix& v);
	FixedMatrix& operator+=(const Fixed& b);

	// subtraction
	FixedMatrix operator-(const FixedMatrix& b);
    friend FixedMatrix operator-(FixedMatrix const&, FixedMatrix const&);
	FixedMatrix& operator-=(const FixedMatrix& b);

    // multiplication
	FixedMatrix operator*(const FixedMatrix& b);
    friend FixedMatrix operator*(FixedMatrix const&, FixedMatrix const&);
    FixedMatrix& operator*=(const FixedMatrix& b);

    // inversion
    FixedMatrix operator!();
    // transpose
    FixedMatrix operator~();

    // indices
    Fixed operator[] (int i);
    Fixed operator() (int mm, int nn);

    // boolean
    bool operator == (const FixedMatrix& b);

private:
	int m;
	int n;
	int size;
	std::vector<Fixed> matrix;
};

#endif /* FIXEDMATRIX_H_ */
