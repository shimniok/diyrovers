#ifndef __UTIL_H
#define __UTIL_H

/** Utility routines */

#define clamp360(x) clamp((x), 0, 360.0, false)
#define clamp180(x) clamp((x), -180.0, 180.0, true)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * Clamp value between min and max. Specify which of the two are inclusive
 * @param v is the value to clamp
 * @param min is the minimum value, inclusive if flip==false, exclusive otherwise
 * @param max is the maximum value, inclusive if flip==true, exclusive otherwise
 * @param flip determines whether min or max is inclusive
 * @return the clamped value
 */
float clamp(float v, float min, float max, bool flip);

/** Convert char to integer */
int ctoi(char c);

/** Convert string to floating point */
double cvstof(char *s);

/** Convert unsigned long to string
 *
 * @param n is the unsigned long to convert
 * @returns char * to static char array
 */
char *cvntos(unsigned long n);

/** Convert signed long to string
 *
 * @param n is the signed long to convert
 * @returns char * to static char array
 */
char *cvitos(long n);

/** Convert float/double to string
 *
 * @param number is the floating point number to convert
 * @param digits is the number of digits after the decimal point
 * @return char * to static char array
 */
char *cvftos(double number, int digits);

/** Tokenize a string 
 * @param s is the string to tokenize
 * @param max is the maximum number of characters
 * @param delim is the character delimiter on which to tokenize
 * @returns t is the pointer to the next token
 */
char *split(char *s, char *t, int max, char delim);

#endif
