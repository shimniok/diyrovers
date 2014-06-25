#include "SystemState.h"
#include "globals.h"
#include "logging.h"
#include "util.h"
#include "SDFileSystem.h"
#include "SerialGraphicLCD.h"

// TODO 2 set up logging out of low priority interrupt handler

//SDFileSystem sd(p5, p6, p7, p8, "log"); // mosi, miso, sclk, cs
static FILE *logp;

Timer logtimer;
//extern int bufCount;

// If I use arduino style print routines, logging takes ~1000 / ~8000 usec
// the big sprintf takes ~ 700-750 usec all by itself
void logData( SystemState *s )
{
	//char buf[256];
	unsigned int t1, t2;
	logtimer.start();
	logtimer.reset();
	t1 = logtimer.read_us();

	if (s) {
		fputs(cvitos(s->millis), logp);
		fputc(',',logp);
		fputs(cvftos(s->current, 2), logp);
		fputc(',',logp);
		fputs(cvftos(s->voltage, 2), logp);
		fputc(',',logp);
		for (int q=0; q < 3; q++) {
			fputs(cvftos(s->gyro[q], 6), logp);
			fputc(',',logp);
		}
		fputs(cvitos(s->gTemp), logp);
		fputc(',',logp);
		for (int q=0; q < 3; q++) {
			fputs(cvitos(s->a[q]), logp);
			fputc(',',logp);
		}
		/*
		for (int q=0; q < 3; q++) {
			printInt(logp, s->m[q]);
			fputc(',',logp);
		}
		*/
		fputs(cvftos(s->gHeading, 2), logp);
		fputc(',',logp);

		// GPS 1
		fputs(cvftos(s->gpsLatitude, 7), logp);
		fputc(',',logp);
		fputs(cvftos(s->gpsLongitude, 7), logp);
		fputc(',',logp);
		//printFloat(logp, s->gpsLatitude, 7);
		//fputc(',',logp);
		//printFloat(logp, s->gpsLongitude, 7);
		//fputc(',',logp);
		fputs(cvftos(s->gpsCourse_deg, 2), logp);
		fputc(',',logp);
		fputs(cvftos(s->gpsSpeed_mps, 2), logp);
		fputc(',',logp);
		fputs(cvftos(s->gpsHDOP, 1), logp);
		fputc(',',logp);
		fputs(cvitos(s->gpsSats), logp);
		fputc(',',logp);
		// Encoders
		fputs(cvftos(s->lrEncDistance, 7), logp);
		fputc(',',logp);
		fputs(cvftos(s->rrEncDistance, 7), logp);
		fputc(',',logp);
		fputs(cvftos(s->lrEncSpeed, 2), logp);
		fputc(',',logp);
		fputs(cvftos(s->rrEncSpeed, 2), logp);
		fputc(',',logp);
		fputs(cvftos(s->encHeading, 2), logp);
		fputc(',',logp);
		// Estimates
		fputs(cvftos(s->estHeading, 2), logp);
		fputc(',',logp);
		fputs(cvftos(s->estLagHeading, 2), logp);
		fputc(',',logp);
		fputs(cvftos(s->gbias, 6), logp);
		fputc(',',logp);
		fputs(cvftos(s->estLatitude,  7), logp);
		fputc(',',logp);
		fputs(cvftos(s->estLongitude, 7), logp);
		fputc(',',logp);
		fputs(cvftos(s->estX, 4), logp);
		fputc(',',logp);
		fputs(cvftos(s->estY, 4), logp);
		fputc(',',logp);
		// Nav
		fputs(cvitos(s->nextWaypoint), logp);
		fputc(',',logp);
		fputs(cvftos(s->bearing, 2), logp);
		fputc(',',logp);
		fputs(cvftos(s->distance, 3), logp);
		fputc(',',logp);
		fputs(cvftos(s->steerAngle, 3), logp);
		fputc(',',logp);
		fputs(cvftos(s->errHeading, 3), logp);
		fputc(',',logp);
		fputs(cvftos(s->LABrg, 2), logp);
		fputc(',',logp);
		fputs(cvftos(s->LAx, 4), logp);
		fputc(',',logp);
		fputs(cvftos(s->LAy, 4), logp);
		fputc('\n',logp);
		fflush(logp);

		t2 = logtimer.read_us();
		fputs(cvitos(t2-t1), stdout);
		fputs(" us\n", stdout);
	}

    return;
}


FILE *openlog(const char *prefix)
{
    FILE *fp = 0;
    char myname[64];

    pc.printf("Opening file...\n");

    while (fp == 0) {
    	sprintf(myname, "%s/test.txt", LOGDIR);
        if ((fp = fopen(myname, "w")) == 0) {
            pc.printf("Waiting for filesystem to come online...");
            wait(0.200);
            lcd.pos(0,1);
            lcd.printf("%-16s", "Waiting for fs");
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
        pc.printf("file write failed: %s\n", myname);
    } else {

        // TODO 3 set error message, get rid of writing to terminal

        //status = true;
        pc.printf("opened %s for writing\n", myname);
        lcd.pos(0,1);
        lcd.printf("%-16s", myname);
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
