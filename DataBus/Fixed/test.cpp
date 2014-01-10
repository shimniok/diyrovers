#include "Fixed.h"
#include "FixedMatrix.h"

using namespace std;

int main() {
	Fixed a, b;
	FixedMatrix x(2,2);
	FixedMatrix y(2,2);

	a = 2.2;
	b = 4.4;

	for (int i=0; i < 4; i++) {
		x[i] = i;
		y[i] = i;
	}

}
