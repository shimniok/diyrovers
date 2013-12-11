#ifndef __DEBUG_H
#define __DEBUG_H

/** Debugging utilities */

#include <stdio.h>

#define WHERE(x) fprintf(stdout, "%d\n", __LINE__);

#endif