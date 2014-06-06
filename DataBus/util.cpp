#include <math.h>
#include <string.h>

#define MAXBUF 32
#define MAXDIGITS 10

/** 
 * Clamp a value (angle) between min (non-inclusive) and max (inclusive)
 * e.g. clamp(v, 0, 360) or clamp(v, -180, 180)
 */
float clamp(float v, float min, float max, bool flip) 
{
    float i;
    float f;
    float mod = (max - min);

    f = modff((v/mod), &i) * mod;
    if (flip) {
        if (f > max) f -= mod;
        if (f <= min) f += mod;
    } else {
        if (f < min) f += mod;
        if (f >= max) f -= mod;
    }
    return f;
}

// convert character to an int
//
int ctoi(char c)
{
  int i=-1;
  
  if (c >= '0' && c <= '9') {
    i = c - '0';
  }

  //printf("char: %c  int %d\n", c, i); 
 
  return i;
}


// convert string to floating point
//
double cvstof(char *s)
{
  double f=0.0;
  double mult = 0.1;
  bool neg = false;
  //char dec = 1;
  
  // leading spaces
  while (*s == ' ' || *s == '\t') {
    s++;
    if (*s == 0) break;
  }

  // What about negative numbers?
  if (*s == '-') {
    neg = true;
    s++;
  }

  // before the decimal
  //
  while (*s != 0) {
    if (*s == '.') {
      s++;
      break;
    }
    f = (f * 10.0) + (double) ctoi(*s);
    s++;
  }
  // after the decimal
  while (*s != 0 && *s >= '0' && *s <= '9') {
    f += (double) ctoi(*s) * mult;
    mult /= 10;
    s++;
  }
  
  // if we were negative...
  if (neg) f = -f;
  
  return f;
}


char *cvntos(unsigned long n)
{
	static char buf[MAXBUF+1]; // +1 null termination
	char *str = buf+MAXBUF;
	int i;

	// ensure null termination
	*str-- = '\0';

	for (i = 0; i < MAXBUF; i++) {
        unsigned long m = n;
        n /= 10;
        char c = m - 10 * n;
        *--str = c + '0';
        if (n == 0) break;
    }

    return str;
}



char *cvitos(long n)
{
	static char buf[MAXBUF+2]; // +1 for sign, +1 for null termination
	char *str = buf;

	*str = '\0';
    if (n < 0) {
        *str++ = '-';
        *str = '\0';
        n = -n;
    }
    strcat(str, cvntos(n));

    return buf;
}



char *cvftos(double number, int digits)
{
	static char buf[MAXBUF+3+MAXDIGITS]; // +1 for termination, +1 for sign, +1 for ., +MAXDIGITS for digits
	char *str = buf;
	int i;

	// ensure proper null termination
	*str = '\0';

	// Limited buffer space for decimals
	if (digits > MAXDIGITS)
		digits = MAXDIGITS;

    if (isnan(number)) {
    	strcpy(buf, "nan");
    } else if (isinf(number)) {
    	strcpy(buf, "inf");
    } else if (number > 4294967040.0 || number < -4294967040.0) {  // constant determined empirically
    	strcpy(buf, "ovf");
    } else {

		// Handle negative numbers
		if (number < 0.0) {
			// Add the sign
			strcat(str, "-");
			number = -number;
		}

		// Round correctly so that print(1.999, 2) prints as "2.00"
		double rounding = 0.5;
		for (i=0; i < digits; i++)
			rounding /= 10.0;
		number += rounding;

		// Extract the integer part of the number and print it
		unsigned long int_part = (unsigned long)number;
		double remainder = number - (double)int_part;

		// Add the integer part
		strcat(str, cvntos(int_part));
		// Add the decimal point
		char *s = str + strlen(str);
		*s++ = '.';

		// Extract digits from the remainder one at a time
		while (digits-- > 0) {
			remainder *= 10.0;
			int toPrint = (int) remainder;
			*s++ = toPrint + '0';
			remainder -= (double) toPrint;
		}
		*s = '\0';
    }

    return str;
}


// copy t to s until delimiter is reached
// return location of delimiter+1 in t
// if s or t null, return null
char *split(char *s, char *t, int max, char delim)
{
  int i = 0;

  if (s == 0 || t == 0)
    return 0;

  while (*t != 0 && *t != '\n' && *t != delim && i < max) {
    *s++ = *t++;
    i++;
  }
  *s = 0;

  return t+1;
}
