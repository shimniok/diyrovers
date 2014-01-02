#include <stdint.h>
#include <stdio.h>

#define BAMBITS 16 // must be multiple of 8

typedef uint16_t bam;

bam add(bam first, bam second) {
  return (first + second);
}

float sub(bam first, bam second) {
  return (first - second);
}

float bamToDegrees() {
}

bam degreesToBAM(float degrees) {
  float x;
  uint32_t y;
  bam result;

  // normalize degrees to 0..360
  if (degrees >= 360.0) {
    degrees = 0;
  } else if (degrees < 0) {
    degrees += 360;
  }

  result = 0;
  for (x = 360.0, y = (1<<BAMBITS); y != 0; x /= 2, y >>= 1) {
    if (degrees > x) {
      result += y;
      degrees -= x;
    }
}

  return result;
}

int main() {
  float x;

  for (x=0; x < 360; x += 0.01) {
    fprintf(stdout, "%5.2f %x\n", x, degreesToBAM(x));
  }

}
