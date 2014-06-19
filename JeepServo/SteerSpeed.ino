
#define MAXDIFF 20

// Maps difference in target and position to a steering speed
// Have to tune for reasonably close tracking to target and for
// stability.
const static int speed[MAXDIFF+1] = {
    0, // 0 
   50, // 1
   60, // 2
   65, // 3
   70, // 4
   75, // 5
   80, // 6
   85, // 7
   90, // 8
   95, // 9
  100, // 10
  110, // 11
  120, // 12
  130, // 13
  140, // 14
  150, // 15
  160, // 16
  170, // 17
  180, // 18
  190, // 19
  200, // 20
};

/** Convert potentiometer difference to steering speed
 */
int diffToSteerSpeed(int diff) {
  int steerSpeed = 0;
  
  if (diff < 1) diff = -diff;
  
  if (diff > MAXDIFF) {
    steerSpeed = speed[MAXDIFF];
  } else {
    steerSpeed = speed[diff];
  }
    
  return steerSpeed;
}
