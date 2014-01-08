#include <stdint.h>
#include <stdio.h>

typedef uint32_t bam;
typedef uint64_t longbam;
const float conv = (1<<28)/22.5;
const float iconv = 22.5/(1<<28);

inline float clamp360(float d) {
	while (d < 0) d += 360.0;
	while (d >= 360) d -= 360.0;
	return d;
}

inline bam add(bam first, bam second) {
	return (first + second);
}

inline float sub(bam first, bam second) {
	return (first - second);
}

float bamToDegrees(bam b) {
	float result;

	return b * iconv;
}

bam degreesToBAM(float d) {
  return d * conv;
}

int main() {
  float x, y, z;
  bam bx, by;
  longbam bz;
  char c;

  while (!feof(stdin)) {

	  fscanf(stdin, "%f %f %c", &x, &y, &c);

	  bx=degreesToBAM(x);
	  by=degreesToBAM(y);

	  switch (c) {
	  case '+':
		  bz = bx + by;
		  z = clamp360(x + y);
		  printf("bam: 0x%08x %c 0x%08x == 0x%08x\n", bx, c, by, (bam) bz);
		  break;
	  case '-':
		  bz = bx - by;
		  z = clamp360(x - y);
		  printf("bam: 0x%08x %c 0x%08x == 0x%08x\n", bx, c, by, (bam) bz);
		  break;
	  case '*':
		  bz = degreesToBAM(x * y);
		  z = clamp360(x * y);
		  printf("bam: %f %c %f == 0x%08x\n", x, c, y, (bam) bz);
		  break;
	  case '/':
		  bz = bx / y;
		  z = clamp360(x / y);
		  printf("bam: %f %c %f == 0x%08x\n", x, c, y, (bam) bz);
		  break;
	  }

	  printf("float: %.2f %c %.2f == %.4f\n", x, c, y, z);
	  printf("bam2f: %.2f %c %.2f == %.4f\n", bamToDegrees(bx), c, bamToDegrees(by), bamToDegrees(bz));
	  printf("err == %.4f\n", 1-bamToDegrees(bz)/z);
  }

  return 0;
}
