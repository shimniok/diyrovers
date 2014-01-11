/*
* FixedMatrix.cpp
*
*  Created on: Jan 9, 2014
*      Author: mes
*/

#include "FixedMatrix.h"
#include "Fixed.h"

FixedMatrix::FixedMatrix(const FixedMatrix &nfm):
	m(nfm.m)
,	n(nfm.n)
,	size(nfm.m*nfm.n)
{
	matrix = nfm.matrix;
}

/* nn = columns
 * mm = rows
 */
FixedMatrix::FixedMatrix(const int mm, const int nn):
	m(mm)
,	n(nn)
,	size(m*n)
{
	matrix = std::vector<Fixed>(size, 0);
}

FixedMatrix::~FixedMatrix() {
	delete &m;
	delete &n;
	delete &matrix;
	delete this;
}

// assignment
FixedMatrix& FixedMatrix::operator=(const FixedMatrix &b) {
    // check for self-assignment by comparing the address of the
    // implicit object and the parameter
    if (this == &b)
        return *this;

	matrix = b.matrix;
	m = b.m;
	n = b.n;
	return *this;
}

// addition
FixedMatrix FixedMatrix::operator+(const FixedMatrix& b) {
	assert(m == b.m);
	assert(n == b.n);
	FixedMatrix result(m,n);
	for (int i=0; i < size; i++)
		result.matrix[i] = matrix[i] + b.matrix[i];
	return result;
}

FixedMatrix FixedMatrix::operator+(const Fixed& b) {
	FixedMatrix result(m,n);
	for (int i=0; i < size; i++)
		result.matrix[i] += b;
	return result;
}

FixedMatrix operator+(FixedMatrix const& a, FixedMatrix const& b) {
	assert(a.m == b.m);
	assert(a.n == b.n);
	FixedMatrix result(a);
	result += b;
	return result;
}


FixedMatrix& FixedMatrix::operator+=(const FixedMatrix& b) {
	assert(m == b.m);
	assert(n == b.n);
	for (int i=0; i < size; i++)
		matrix[i] += b.matrix[i];
	return *this;
}

FixedMatrix& FixedMatrix::operator+=(const Fixed& b) {
	for (int i=0; i < size; i++)
		matrix[i] += b;
	return *this;
}

// subtraction
FixedMatrix FixedMatrix::operator-(const FixedMatrix& b) {
	assert(m == b.m);
	assert(n == b.n);
	FixedMatrix result(m,n);
	for (int i=0; i < size; i++)
		result.matrix[i] = matrix[i] - b.matrix[i];
	return result;
}

FixedMatrix operator-(FixedMatrix const& a, FixedMatrix const& b) {
	assert(a.m == b.m);
	assert(a.n == b.n);
	FixedMatrix result(a);
	result -= b;
	return result;
}

FixedMatrix& FixedMatrix::operator-=(const FixedMatrix& b) {
	assert(m == b.m);
	assert(n == b.n);
	for (int i=0; i < size; i++)
		matrix[i] -= b.matrix[i];
	return *this;
}

// multiplication
FixedMatrix FixedMatrix::operator*(const FixedMatrix& b) {
	assert(n == b.m);
	FixedMatrix result(m, b.n);
    for (int i=0; i < m; i++) {
        for(int j=0; j < b.n; j++) {
            result.matrix[b.n*i+j] = 0;
            for (int k=0; k < n; k++) {
                result.matrix[i*b.n+j] += matrix[i*n+k] * b.matrix[k*b.n+j];
            }
        }
    }
	return result;
}

FixedMatrix operator*(FixedMatrix const& a, FixedMatrix const& b) {
	assert(a.m == b.m);
	assert(a.n == b.n);
	FixedMatrix result(a);
	result *= b;
	return result;
}

FixedMatrix& FixedMatrix::operator*=(const FixedMatrix& b) {
	FixedMatrix result(*this);
	assert(n == b.m);
    for (int i=0; i < m; i++) {
        for(int j=0; j < b.n; j++) {
            result.matrix[b.n*i+j] = 0;
            for (int k=0; k < n; k++) {
                result.matrix[i*b.n+j] += matrix[i*n+k] * b.matrix[k*b.n+j];
            }
        }
    }
    *this = result;
    return *this;
}

// inversion
FixedMatrix FixedMatrix::operator!() {
    // A = input matrix AND result matrix
    // n = number of rows = number of columns in A (n x n)
    int pivrow=0;   // keeps track of current pivot row
    int k,i,j;        // k: overall index along diagonal; i: row index; j: col index
    int pivrows[n]; // keeps track of rows swaps to undo at end
    Fixed tmp;        // used for finding max value and making column swaps

    for (k = 0; k < n; k++) {
        // find pivot row, the row with biggest entry in current column
        tmp = 0.0;
        for (i = k; i < n; i++) {
            if (fabs(matrix[i*n+k]) >= tmp) {    // 'Avoid using other functions inside abs()?'
                tmp = fabs(matrix[i*n+k]);
                pivrow = i;
            }
        }

        // check for singular matrix
        assert(matrix[pivrow*n+k] != 0.0f);

        // Execute pivot (row swap) if needed
        if (pivrow != k) {
            // swap row k with pivrow
            for (j = 0; j < n; j++) {
                tmp = matrix[k*n+j];
                matrix[k*n+j] = matrix[pivrow*n+j];
                matrix[pivrow*n+j] = tmp;
            }
        }
        pivrows[k] = pivrow;    // record row swap (even if no swap happened)

        tmp = 1.0f/matrix[k*n+k];    // invert pivot element
        matrix[k*n+k] = 1.0f;        // This element of input matrix becomes result matrix

        // Perform row reduction (divide every element by pivot)
        for (j = 0; j < n; j++) {
        	matrix[k*n+j] = matrix[k*n+j]*tmp;
        }

        // Now eliminate all other entries in this column
        for (i = 0; i < n; i++) {
            if (i != k) {
                tmp = matrix[i*n+k];
                matrix[i*n+k] = 0.0f;  // The other place where in matrix becomes result mat
                for (j = 0; j < n; j++) {
                	matrix[i*n+j] = matrix[i*n+j] - matrix[k*n+j]*tmp;
                }
            }
        }
    }

    // Done, now need to undo pivot row swaps by doing column swaps in reverse order
    for (k = n-1; k >= 0; k--) {
        if (pivrows[k] != k) {
            for (i = 0; i < n; i++) {
                tmp = matrix[i*n+k];
                matrix[i*n+k] = matrix[i*n+pivrows[k]];
                matrix[i*n+pivrows[k]] = tmp;
            }
        }
    }

	return *this;
}

// transpose
FixedMatrix FixedMatrix::operator~() {
	FixedMatrix result(n, m);
	for (int i=0; i < n; i++) {
		for (int j=0; j < m; j++) {
			result.matrix[j*n+i] = matrix[i*m+j];
		}
	}
	return result;
}

// indices
Fixed FixedMatrix::operator[] (int i) {
	return matrix[i];
}

Fixed FixedMatrix::operator() (int mm, int nn) {
	return matrix[n*mm+nn];
}

bool FixedMatrix::operator == (const FixedMatrix& b) {
	bool result = true;
	for (int i = 0; i < size; i++) {
		if (matrix[i] != b.matrix[i]) {
			result = false;
			break;
		}
	}
	return result;
}
