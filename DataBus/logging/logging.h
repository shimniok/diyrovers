#ifndef __LOGGING_H
#define __LOGGING_H

/** Data logging functions */

#include "mbed.h"
#include "SystemState.h"

#define LOGDIR "/log"

FILE *openlog(const char *prefix);
bool initLogfile(void);
void logData( SystemState *s );
void closeLogfile(void);

#endif
