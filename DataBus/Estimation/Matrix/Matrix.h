#ifndef __DCM_MATRIX_H
#define __DCM_MATRIX_H

/** Matrix math library */

#define SINGULAR_MATRIX     (1<<0)
#define ERR1                (1<<1)
#define ERR2                (1<<2)
#define ERR3                (1<<3)

/** Adds two n x m matrices: C = A + B
 * @param A is the first matrix
 * @param B is the second matrix
 * @param n is the rows of A, B, C
 * @param m is the columns of A, B, C
 * @returns C is the result of the addition
 */
void Matrix_Add(int n, int m, float *C, float *A, float *B);

/** Subtracts two n x m matrices: C = A - B
 * @param A is the first matrix
 * @param B is the second matrix
 * @param n is the rows of A, B, C
 * @param m is the columns of A, B, C
 * @returns C is the result of the addition
 */
void Matrix_Subtract(int n, int m, float *C, float *A, float *B);

/** Multiplies an n x m matrix with an m x p matrix to get a third n x p matrix: C = A * B
 * @param A is the first matrix
 * @param B is the second matrix
 * @param n is the rows of A
 * @param p is the columns of A and rows of B
 * @param m is the columns of B
 * @returns C as the result of the mutliplication
 */
void Matrix_Multiply(int n, int m, int p, float *C, float *A, float *B);

/** Transposes an n x m matrix and places the result in C: C = A.T
 * @param n is the rows
 * @param m is the columns
 * @param C is the result matrix
 * @param A is the matrix to transpose
 * @returns C
 */
void Matrix_Transpose(int n, int m, float *C, float *A);

/** Inverses a matrix
 * @param n is the number of rows and columns (n x n)
 * @param A is the matrix to inverse
 * @returns A
 */
void Matrix_Inverse(int n, float *A);

/** Copies one matrix into another
 * @param n is the number of rows
 * @param m is the number of columns
 * @param A is the matrix being copied
 * @param C is the matrix into which A is copied
 * @returns C
 */
void Matrix_Copy(int n, int m, float *C, float *A);

#endif
