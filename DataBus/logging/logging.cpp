#include "print.h"
#include "SystemState.h"
#include "logging.h"
#include "SDFileSystem.h"
#include "SerialGraphicLCD.h"

// TODO 2 figure out a way to pull out status updates, some kind of message, queue, something.
extern SerialGraphicLCD lcd;

// TODO 2 set up logging out of low priority interrupt handler

#define LOGDIR "/log" // TODO 2 parameterize log directory

//SDFileSystem sd(p5, p6, p7, p8, "log"); // mosi, miso, sclk, cs
static FILE *logp;

Timer logtimer;

// If I use arduino style print routines, logging takes ~1000 / ~8000 usec
// the big sprintf takes ~ 700-750 usec all by itself
void logData( SystemState *s )
{
	//char buf[256];
	//unsigned int t1, t2;
	//logtimer.start();
	//logtimer.reset();
	//t1 = logtimer.read_us();

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
		printFloat(logp, s->bearing, 2);
		fputc(',',logp);
		printFloat(logp, s->distance, 3);
		fputc(',',logp);
		printFloat(logp, s->steerAngle, 3);
		fputc(',',logp);
		printFloat(logp, s->errHeading, 3);
		fputc(',',logp);
		fputc('\n',logp);
		fflush(logp);

		//t2 = logtimer.read_us();
		//fprintf(stdout, "%d\n", t2-t1);
	}

    return;
}


FILE *openlog(const char *prefix)
{
    FILE *fp = 0;
    char myname[64];

    fputs("Opening file...\n", stdout);

    while (fp == 0) {
    	sprintf(myname, "%s/test.txt", LOGDIR);
        if ((fp = fopen(myname, "w")) == 0) {
            fputs("Waiting for filesystem to come online...", stdout);
            wait(0.200);
            lcd.pos(0,1);
            lcd.puts("Waiting for fs  ");
        }
    }
    fclose(fp);

    for (int i = 0; i < 1000; i++) {
        sprintf(myname, "%s/%s%03d.csv", LOGDIR, prefix, i);
        if ((fp = fopen(myname, "r")) == 0) {
            break;
        } else {
            fclose(fp);
        }
    }
    fp = fopen(myname, "w");
    if (fp == 0) {
    	fputs("file write failed: ", stdout);
    	fputs(myname, stdout);
    	fputc('\n', stdout);
    } else {

        // TODO 3 set error message, get rid of writing to terminal

        //status = true;
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
        //fprintf(logp, "s.millis, s.current, s.voltage, s.gx, s.gy, s.gz, s.gTemp, s.ax, s.ay, s.az, s.mx, s.my, s.mz, s.gHeading, s.cHeading, s.roll, s.pitch, s.yaw, s.gpsLatitude, s.gpsLongitude, s.gpsCourse, s.gpsSpeed, s.gpsHDOP, s.lrEncDistance, s.rrEncDistance, s.lrEncSpeed, s.rrEncSpeed, s.encHeading, s.estHeading, s.estLatitude, s.estLongitude, s.estNorthing, s.estEasting, s.estX, s.estY, s.nextWaypoint, s.bearing, s.distance, s.gbias, s.errAngle, s.leftRanger, s.rightRanger, s.centerRanger, s.crossTrackErr\n");
    }

    return status;
}

void closeLogfile(void)
{
    if (logp) fclose(logp);
}
