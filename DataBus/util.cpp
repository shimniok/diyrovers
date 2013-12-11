#include <math.h>

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
