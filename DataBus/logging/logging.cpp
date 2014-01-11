#include "print.h"
#include "SystemState.h"
#include "logging.h"
#include "SDFileSystem.h"
#include "SerialGraphicLCD.h"

// TODO 2 set up logging as a low priority task
extern SerialGraphicLCD lcd;

#define LOGDIR "/log"

static FILE *logp;

Timer logtimer;

// If I use arduino style print routines, logging takes ~1000 / ~8000 usec
// the big sprintf takes ~ 700-750 usec all by itself
void logData( SystemState *s )
{
	if (s) {
		printInt(logp, s->millis);
		fputc(',',logp);
		printFloat(logp, s->current, 2);
		fputc(',',logp);
		printFloat(logp, s->voltage, 2);
		fputc(',',logp);
		for (int q=0; q < 3; q++) {
			printFloat(logp, s->gyro[q], 6);
			fputc(',',logp);
		}
		printInt(logp, s->gTemp);
		fputc(',',logp);
		for (int q=0; q < 3; q++) {
			printInt(logp, s->a[q]);
			fputc(',',logp);
		}
		/*
		for (int q=0; q < 3; q++) {
			printInt(logp, s->m[q]);
			fputc(',',logp);
		}
		*/
		printFloat(logp, s->gHeading, 2);
		fputc(',',logp);

		// GPS 1
		printFloat(logp, s->gpsLatitude, 7);
		fputc(',',logp);
		printFloat(logp, s->gpsLongitude, 7);
		fputc(',',logp);
		printFloat(logp, s->gpsCourse_deg, 2);
		fputc(',',logp);
		printFloat(logp, s->gpsSpeed_mps, 2);
		fputc(',',logp);
		printFloat(logp, s->gpsHDOP, 1);
		fputc(',',logp);
		printInt(logp, s->gpsSats);
		fputc(',',logp);
		// Encoders
		printFloat(logp, s->lrEncDistance, 7);
		fputc(',',logp);
		printFloat(logp, s->rrEncDistance, 7);
		fputc(',',logp);
		printFloat(logp, s->lrEncSpeed, 2);
		fputc(',',logp);
		printFloat(logp, s->rrEncSpeed, 2);
		fputc(',',logp);
		printFloat(logp, s->encHeading, 2);
		fputc(',',logp);
		// Estimates
		printFloat(logp, s->estHeading, 2);
		fputc(',',logp);
		printFloat(logp, s->estLagHeading, 2);
		fputc(',',logp);
		printFloat(logp, s->estLatitude,  7);
		fputc(',',logp);
		printFloat(logp, s->estLongitude, 7);
		fputc(',',logp);
		printFloat(logp, s->estX, 4);
		fputc(',',logp);
		printFloat(logp, s->estY, 4);
		fputc(',',logp);
		// Nav
		printInt(logp, s->nextWaypoint);
		fputc(',',logp);
		printFloat(logp, s->bearing_deg, 2);
		fputc(',',logp);
		printFloat(logp, s->distance_m, 3);
		fputc(',',logp);
		printFloat(logp, s->steerAngle, 3);
		fputc(',',logp);
		printFloat(logp, s->errHeading, 3);
		fputc(',',logp);
		fputc('\n',logp);
		fflush(logp);
	}

    return;
}


// TODO 3 better decoupling for openlog, print to screen?? lcd?? should probably send event to status task

FILE *openlog(const char *prefix)
{
    FILE *fp = 0;
    char myname[32];

    fputs("Opening file...\n", stdout);
	strcpy(myname, LOGDIR);
	strcat(myname, "/test.txt");
    while (fp == 0) {
        if ((fp = fopen(myname, "w")) == 0) {
            fputs("Waiting for filesystem to come online...", stdout);
            wait(0.200);
            lcd.pos(0,1);
            lcd.puts("Waiting for fs  ");
        }
    }
    fclose(fp);

    char n[4];
    n[3] = 0;

    for (n[0] = '0'; n[0] <= '9'; n[0]++) {
        for (n[1] = '0'; n[1] <= '9'; n[1]++) {
            for (n[2] = '0'; n[2] <= '9'; n[2]++) {

				strcpy(myname, LOGDIR);
				strcat(myname, "/");
				strcat(myname, prefix);
				strcat(myname, n);

				//sprintf(myname, "%s/%s%03d.csv", LOGDIR, prefix, i);
				if ((fp = fopen(myname, "r")) == 0) {
					break;
				} else {
					fclose(fp);
				}
            }
        }

    }
    fp = fopen(myname, "w");
    if (fp == 0) {
    	fputs("file write failed: ", stdout);
    	fputs(myname, stdout);
    	fputc('\n', stdout);
    } else {
        fputs("opened ", stdout);
        fputs(myname, stdout);
        fputs(" for writing\n", stdout);
        lcd.pos(0,1);
        int pad=16-strlen(myname);
        lcd.puts(myname);
        while(pad--) lcd.putc(' ');
    }

    return fp;
}


// Find the next unused filename of the form logger##.csv where # is 0-9
//
bool initLogfile()
{
    bool status = false;

    logp = openlog("log");

    if (logp != 0) {
        status = true;
    }

    return status;
}

void closeLogfile(void)
{
    if (logp) fclose(logp);
}
