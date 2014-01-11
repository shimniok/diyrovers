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
int ctoi(const char c);

/** Convert int to string */
char *itoa(const int i);

/** Convert string to floating point */
double cvstof(const char *s);

/** Tokenize a string 
 * @param s is the string to tokenize
 * @param max is the maximum number of characters
 * @param delim is the character delimiter on which to tokenize
 * @returns t is the pointer to the next token
 */
char *split(char *s, char *t, int max, char delim);

#endif
