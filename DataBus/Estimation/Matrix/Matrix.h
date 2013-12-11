#ifndef __DCM_MATRIX_H
#define __DCM_MATRIX_H

/** Matrix math library */

#define SINGULAR_MATRIX     (1<<0)
#define ERR1                (1<<1)
#define ERR2                (1<<2)
#define ERR3                (1<<3)

/** Take cross product of two 3x1 vectors: v1 x v2 = vectorOut
 * @param v1 is the first vector
 * @param v2 is the second vector
 * @returns vectorOut = v1 x v2
 */
void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]);

/** Multiple 3x1 vector by scalar: vectorOut = vectorIn times scale2
 * @param vectorIn the vector
 * @param scale2 is the scalar
 * @returns vectorOut the result
 */
void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2);

/** TDot product of two 3x1 vectors vector1 . vector2
 * @param vector1 is the first vector
 * @param vector2 is the second vector
 * @returns float result of dot product
 */
float Vector_Dot_Product(float vector1[3], float vector2[3]);

/** Adds two 3x1 vectors: vectorOut = vectorIn1 + vectorIn2
 * @param vectorIn1 is the first vector
 * @param vectorIn2 is the second vector
 * @returns vectorOut is the result of the addition
 */
void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);

/** Adds two 3x3 matrices: C = A + B
 * @param A is the first vector
 * @param B is the second vector
 * @returns C is the result of the addition
 */
void Matrix_Add(float C[3][3], float A[3][3], float B[3][3]);

/** Adds two n x m matrices: C = A + B
 * @param A is the first vector
 * @param B is the second vector
 * @param n is the rows of A, B, C
 * @param m is the columns of A, B, C
 * @returns C is the result of the addition
 */
void Matrix_Add(int n, int m, float *C, float *A, float *B);

void Matrix_Subtract(int n, int m, float *C, float *A, float *B);

/** Multiplies two 3x3 matrices to get a third 3x3 matrix: c = ab
 * @param a is the first vector
 * @param b is the second vector
 * @returns c as the result of the mutliplication
 */
void Matrix_Multiply(float c[3][3], float a[3][3], float b[3][3]);

/** Multiplies two 3x3 matrices to get a third 3x3 matrix: c = ab
 * @param A is the first matrix
 * @param B is the second matrix
 * @param n is the rows of A
 * @param p is the columns of A and rows of B
 * @param m is the columns of B
 * @returns C as the result of the mutliplication
 */
void Matrix_Multiply(int n, int m, int p, float *C, float *A, float *B);

void Matrix_Transpose(int n, int m, float *C, float *A);

void Matrix_Inverse(int n, float *A);

void Matrix_Inverse(int n, float *C, float *A);

void Matrix_Copy(int n, int m, float *C, float *A);

/** Prints out an m x m matrix
 * @param A is the matrix to print
 * @param name is the name to print out along with the vector
 */
void Matrix_print(int n, int m, float *A, const char *name);

/** Prints out a 1 x 3 vector
 * @param A is the 1 x 3 vector to print
 * @param name is the name to print out along with the vector
 */
void Vector_Print(float A[3], const char *name);

#endif