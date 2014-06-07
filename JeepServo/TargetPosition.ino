#define M1 386L
#define B1 -19000L
#define M2 526L
#define B2 -229000L

/* Convert pulse width to target position (raw ADC read value)
 * @param pw is the pulse width value in ms
 */
long pulseWidthToPosition(long pw) {
  long pos = CENTER;

  // Dual slope interpolation
  if (pw < 1500) {
    pos = M1*pw + B1;
  } else {
    pos = M2*pw + B2;
  }
  pos /= 1000;

//  if (pos > FULL_LEFT) pos = FULL_LEFT;
//  if (pos < FULL_RIGHT) pos = FULL_RIGHT;
  
  return pos;
}

