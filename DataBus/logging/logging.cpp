#include "logging.h"
#include "SDHCFileSystem.h"
#include "SerialGraphicLCD.h"

extern Serial pc;
extern SerialGraphicLCD lcd;

#define LOGDIR "/log"

//SDFileSystem sd(p5, p6, p7, p8, "log"); // mosi, miso, sclk, cs
static FILE *logp;

void clearState( SystemState *s )
{
    s->millis = 0;
    s->current = s->voltage = 0.0;
    s->g[0] = s->g[1] = s->g[2] = 0;
    s->gyro[0] = s->gyro[1] = s->gyro[2] = 0;
    s->gTemp = 0;
    s->a[0] = s->a[1] = s->a[2] = 0;
    s->m[0] = s->m[1] = s->m[2] = 0;
    s->gHeading = s->cHeading = 0.0;
    //s->roll = s->pitch = s->yaw =0.0;
    s->gpsLatitude = s->gpsLongitude = s->gpsCourse_deg = s->gpsSpeed_mps = s->gpsHDOP = 0.0;
    //s->gpsLatitude2 = s->gpsLongitude2 = s->gpsCourse_deg2 = s->gpsSpeed_mps2 = s->gpsHDOP2 = 0.0;
    s->lrEncDistance = s->rrEncDistance = 0.0;
    s->lrEncSpeed = s->rrEncSpeed = s->encHeading = 0.0;
    s->estHeading = s->estLatitude = s->estLongitude = 0.0;
    //s->estNorthing = s->estEasting =
    s->estX = s->estY = 0.0;
    s->nextWaypoint = 0;
    s->bearing = s->distance = 0.0;
}

Timer logtimer;
extern int bufCount;

/*
void logData( const SystemState s ) {
    unsigned char buf[512]; // for now we really only need ~256 bytes but in case I add more to state...
    unsigned char *state = (unsigned char *) &s;
    //unsigned int t1, t2, t3;
    //logtimer.start();
    //logtimer.reset();
    if (logp) {
        //t1 = logtimer.read_us();
        encode(state, sizeof(s), buf, 0); // infinite line size
        //t2 = logtimer.read_us();
        fputs((char *) buf, logp);
        fputs("\n", logp);
        bufCount--;
        fprintf(stdout, "bufCount: %d\n", bufCount);
        //t3 = logtimer.read_us();
        //fprintf(stdout, "%d %d\n", t3-t2, t2-t1);
    }
}
*/

// from Arduino source
size_t printNumber(FILE *f, unsigned long n)
{
    char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';

    do {
        unsigned long m = n;
        n /= 10;
        char c = m - 10 * n;
        *--str = c + '0';
    } while(n);

    return fputs(str, f);
}

// from Arduino source
size_t printInt(FILE *f, long n)
{
    int t = 0;
    if (n < 0) {
        t = fputc('-', f);
        n = -n;
    }
    return printNumber(f, n) + t;
}

// from Arduino source
size_t printFloat(FILE *f, double number, uint8_t digits)
{
    size_t n=0;

    if (isnan(number)) return fputs("nan", f);
    if (isinf(number)) return fputs("inf", f);
    if (number > 4294967040.0) return fputs("ovf", f);  // constant determined empirically
    if (number <-4294967040.0) return fputs("ovf", f);  // constant determined empirically

    // Handle negative numbers
    if (number < 0.0) {
        n += fputc('-', f);
        number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i=0; i < digits; ++i)
        rounding /= 10.0;

    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    n += printInt(f, int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        n += fputc('.', f);
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        n += fputc(toPrint+'0', f);
        remainder -= toPrint;
    }

    return n;
}


// If I use arduino style print routines, logging takes ~1000 / ~8000 usec
// the big sprintf takes ~ 700-750 usec all by itself
void logData( const SystemState s )
{
    //char buf[256];
    unsigned int t1, t2;
    logtimer.start();
    logtimer.reset();
    t1 = logtimer.read_us();
    printInt(logp, s.millis);
    fputc(',',logp);
    printFloat(logp, s.current, 2);
    fputc(',',logp);
    printFloat(logp, s.voltage, 2);
    fputc(',',logp);
    for (int q=0; q < 3; q++) {
        printFloat(logp, s.gyro[q], 6);
        fputc(',',logp);
    }
    printInt(logp, s.gTemp);
    fputc(',',logp);
    for (int q=0; q < 3; q++) {
        printInt(logp, s.a[q]);
        fputc(',',logp);
    }
    /*
    for (int q=0; q < 3; q++) {
        printInt(logp, s.m[q]);
        fputc(',',logp);
    }
    */
    printFloat(logp, s.gHeading, 2);
    fputc(',',logp);

    // GPS 1
    fprintf(logp, "%.7f,%.7f,", s.gpsLatitude, s.gpsLongitude);
    //printFloat(logp, s.gpsLatitude, 7);
    //fputc(',',logp);
    //printFloat(logp, s.gpsLongitude, 7);
    //fputc(',',logp);
    printFloat(logp, s.gpsCourse_deg, 2);
    fputc(',',logp);
    printFloat(logp, s.gpsSpeed_mps, 2);
    fputc(',',logp);
    printFloat(logp, s.gpsHDOP, 1);
    fputc(',',logp);
    printInt(logp, s.gpsSats);
    fputc(',',logp);
    // Encoders
    printFloat(logp, s.lrEncDistance, 7);
    fputc(',',logp);
    printFloat(logp, s.rrEncDistance, 7);
    fputc(',',logp);
    printFloat(logp, s.lrEncSpeed, 2);
    fputc(',',logp);
    printFloat(logp, s.rrEncSpeed, 2);
    fputc(',',logp);
    printFloat(logp, s.encHeading, 2);
    fputc(',',logp);
    // Estimates
    printFloat(logp, s.estHeading, 2);
    fputc(',',logp);
    printFloat(logp, s.estLagHeading, 2);
    fputc(',',logp);
    printFloat(logp, s.estLatitude,  7);
    fputc(',',logp);
    printFloat(logp, s.estLongitude, 7);
    fputc(',',logp);
    printFloat(logp, s.estX, 4);
    fputc(',',logp);
    printFloat(logp, s.estY, 4);
    fputc(',',logp);
    // Nav
    printInt(logp, s.nextWaypoint);
    fputc(',',logp);
    printFloat(logp, s.bearing, 2);
    fputc(',',logp);
    printFloat(logp, s.distance, 3);
    fputc(',',logp);
    printFloat(logp, s.steerAngle, 3);
    fputc(',',logp);
    printFloat(logp, s.errHeading, 3);
    fputc(',',logp);
    fputc('\n',logp);

    t2 = logtimer.read_us();
    //fprintf(stdout, "%d\n", t2-t1);

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
