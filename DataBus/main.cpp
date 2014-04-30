/** Code for "Data Bus" UGV entry for Sparkfun AVC 2013
 *  http://www.bot-thoughts.com/
 */

///////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "mbed.h"
#include "boards.h"
#include "globals.h"
#include "Filesystem.h"
#include "SerialMux.h"
#include "Config.h"
#include "Buttons.h"
#include "Display.h"
#include "Menu.h"
#include "GPSStatus.h"
#include "logging.h"
#include "SystemState.h"
#include "shell.h"
#include "Sensors.h"
#include "kalman.h"
#include "Ublox6.h"
#include "PinDetect.h" // TODO this should be broken into .h, .cpp
#include "Actuators.h"
#include "IncrementalEncoder.h"
#include "Steering.h"
#include "Schedule.h"
#include "GeoPosition.h"
#include "Mapping.h"
#include "SimpleFilter.h"
#include "Beep.h"
#include "util.h"
#include "MAVlink/include/mavlink_bridge.h"
#include "updater.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////////////////////////////

#define absf(x) (x *= (x < 0.0) ? -1 : 1)

#define GPS_MIN_SPEED   2.0             // speed below which we won't trust GPS course
#define GPS_MAX_HDOP    2.0             // HDOP above which we won't trust GPS course/position

// Driver configuration parameters
#define SONARLEFT_CHAN   0
#define SONARRIGHT_CHAN  1
#define IRLEFT_CHAN      2
#define IRRIGHT_CHAN     3  
#define TEMP_CHAN        4
#define GYRO_CHAN        5

// Chassis specific parameters
// TODO 1 put WHEEL_CIRC, WHEELBASE, and TRACK in config.txt
#define WHEEL_CIRC 0.321537             // m; calibrated with 4 12.236m runs. Measured 13.125" or 0.333375m circumference
#define WHEELBASE  0.290
#define TRACK      0.280

#define INSTRUMENT_CHECK    0
#define AHRS_VISUALIZATION  1
#define DISPLAY_PANEL       2

///////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
///////////////////////////////////////////////////////////////////////////////////////////////////////

// OUTPUT
DigitalOut confStatus(LED1);            // Config file status LED
DigitalOut logStatus(LED2);             // Log file status LED
DigitalOut gpsStatus(LED3);             // GPS fix status LED
DigitalOut updaterStatus(LED4);         // update loop status LED
//DigitalOut sonarStart(p18);           // Sends signal to start sonar array pings
Display display;                        // UI display
//Beep speaker(p24);                      // Piezo speaker

// INPUT
Menu menu;
Buttons keypad;

// VEHICLE
Steering steerCalc(TRACK, WHEELBASE);   // steering calculator

// COMM
Serial pc(USBTX, USBRX);                // PC usb communications
SerialMux mux(&pc);
SerialGraphicLCD lcd(UART3TX, UART3RX, SD_FW);  // Graphic LCD with summoningdark firmware

// SENSORS
Sensors sensors;                        // Abstraction of sensor drivers
//DCM ahrs;                             // ArduPilot/MatrixPilot AHRS
Serial *dev;                            // For use with bridge

// MISC
FILE *camlog;                           // Camera log

// Configuration
Filesystem fs;							// set up filesystems
Config config;                          // Persistent configuration
                                        // Course Waypoints
                                        // Sensor Calibration
                                        // etc.

// Timing
Timer timer;                            // For main loop scheduling

// GPS Variables
unsigned long age = 0;                  // gps fix age

// schedule for LED warning flasher
Schedule blink;

// Estimation & Navigation Variables
GeoPosition dr_here;                    // Estimated position based on estimated heading
Mapping mapper;

///////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////

void initFlasher(void);
//void initDR(void);
int autonomousMode(void);
void telemetryMode(void);
void mavlinkMode(void);
void servoCalibrate(void);
void serialBridge(Serial &gps);
int instrumentCheck(void);
void displayData(int mode);
int compassCalibrate(void);
int compassSwing(void);
int gyroSwing(void);
int setBacklight(void);
int reverseScreen(void);
float irDistance(const unsigned int adc);
extern "C" void mbed_reset();

// If we don't close the log file, when we restart, all the written data
// will be lost.  So we have to use a button to force mbed to close the
// file and preserve the data.
//

int dummy(void)
{
    return 0;
}


int resetMe()
{
	mbed_reset();
    
    return 0;
}


int main()
{
	//checkit(__FILE__, __LINE__);
	//xTaskCreate( shell, (const signed char * ) "shell", 128, NULL, (tskIDLE_PRIORITY+3), NULL );
    //checkit(__FILE__, __LINE__);
	//vTaskStartScheduler(); // should never get past this line.
	//while(1);

	// Send data back to the PC
    pc.baud(115200);
    fprintf(stdout, "Data Bus 2014\n");
    fflush(stdin);

    // Let's try setting priorities...
    //NVIC_SetPriority(DMA_IRQn, 0);
    NVIC_SetPriority(TIMER3_IRQn, 2);   // updater running off Ticker
    NVIC_SetPriority(EINT0_IRQn, 5);    // wheel encoders
    NVIC_SetPriority(EINT1_IRQn, 5);    // wheel encoders
    NVIC_SetPriority(EINT2_IRQn, 5);    // wheel encoders
    NVIC_SetPriority(EINT3_IRQn, 5);    // wheel encoders
    NVIC_SetPriority(SPI_IRQn, 7);    	// uSD card, logging
    NVIC_SetPriority(UART0_IRQn, 10);   // USB
    NVIC_SetPriority(UART1_IRQn, 10);
    NVIC_SetPriority(UART2_IRQn, 10);
    NVIC_SetPriority(UART3_IRQn, 10);
    NVIC_SetPriority(I2C0_IRQn, 10);    // sensors?
    NVIC_SetPriority(I2C1_IRQn, 10);    // sensors?
    NVIC_SetPriority(I2C2_IRQn, 10);    // sensors?
    NVIC_SetPriority(ADC_IRQn, 10);     // Voltage/current
    NVIC_SetPriority(TIMER0_IRQn, 10); 	// unused(?)
    NVIC_SetPriority(TIMER1_IRQn, 10); 	// unused(?)
    NVIC_SetPriority(TIMER2_IRQn, 10); 	// unused(?)

    // Something here is jacking up the I2C stuff
    // Also when initializing with ESC powered, it causes motor to run which
    // totally jacks up everything (noise?)
    initSteering();
    initThrottle();
    // initFlasher();                       // Initialize autonomous mode flasher

    display.init();
    display.status("Data Bus 2014");

    fprintf(stdout, "Initializing...\n");
    display.status("Initializing");
    wait(0.2);
    
    // Initialize status LEDs
    updaterStatus = 0;
    gpsStatus = 0;
    logStatus = 0;
    confStatus = 0;

    if (!fifo_init()) {
    	error("\n\n%% Error initializing SystemState fifo %%\n");
    }

    fprintf(stdout, "Loading configuration...\n");
    display.status("Load config");
    wait(0.2);
    if (config.load())                          // Load various configurable parameters, e.g., waypoints, declination, etc.
        confStatus = 1;

    sensors.Compass_Calibrate(config.magOffset, config.magScale);
    //pc.printf("Declination: %.1f\n", config.declination);
    pc.printf("Speed: escZero=%d escMax=%d top=%.1f turn=%.1f Kp=%.4f Ki=%.4f Kd=%.4f\n", 
        config.escZero, config.escMax, config.topSpeed, config.turnSpeed, 
        config.speedKp, config.speedKi, config.speedKd);
    pc.printf("Steering: steerZero=%0.2f steerGain=%.1f gainAngle=%.1f\n",
        config.steerZero, config.steerGain, config.steerGainAngle);

    // Convert lat/lon waypoints to cartesian
    mapper.init(config.wptCount, config.wpt);
    for (unsigned int w = 0; w < MAXWPT && w < config.wptCount; w++) {
        mapper.geoToCart(config.wpt[w], &(config.cwpt[w]));
        pc.printf("Waypoint #%d (%.4f, %.4f) lat: %.6f lon: %.6f, topspeed: %.1f, turnspeed: %.1f\n",
                    w, config.cwpt[w].x, config.cwpt[w].y, config.wpt[w].latitude(), config.wpt[w].longitude(),
                    config.topSpeed + config.wptTopSpeedAdj[w], config.turnSpeed + config.wptTurnSpeedAdj[w]);
    }

    // TODO 3 print mag and gyro calibrations

    // TODO 3 remove GPS configuration, all config will be in object itself I think

    display.status("Nav configuration   ");
    steerCalc.setIntercept(config.interceptDist);               // Setup steering calculator based on intercept distance
    pc.printf("Intercept distance: %.1f\n", config.interceptDist);
    pc.printf("Waypoint distance: %.1f\n", config.waypointDist);
    pc.printf("Brake distance: %.1f\n", config.brakeDist);
    pc.printf("Min turn radius: %.3f\n", config.minRadius);

    fputs("Calculating offsets...\n", stdout);
    display.status("Offset calculation  ");
    wait(0.2);
    // TODO 3 Really need to give the gyro more time to settle
    sensors.gps.disable();
    // TODO 1 sensors.Calculate_Offsets();

    fputs("Starting GPS...\n", stdout);
    display.status("Start GPS           "); // TODO 3: would be nice not to have to pad at this level
    wait(0.2);
    sensors.gps.setUpdateRate(10);
    sensors.gps.enable();

    // TODO Running this seems to prevent the main loop from running ?!
    fputs("Starting Scheduler...\n", stdout);
    display.status("Start scheduler     ");
    wait(0.2);
    // Startup sensor/AHRS ticker; update every UPDATE_PERIOD
    restartNav();
    startUpdater();

/*
    fprintf(stdout, "Starting Camera...\n");
    display.status("Start Camera        ");
    wait(0.5);
    cam.start();
*/

    fprintf(stdout, "Starting keypad...\n");

    keypad.init();
    
    fprintf(stdout, "Adding menu items...\n");

    // Setup LCD Input Menu
    menu.add("Auto mode", &autonomousMode);
    menu.add("Instruments", &instrumentCheck);
    menu.add("Calibrate", &compassCalibrate);
    menu.add("Compass Swing", &compassSwing);
    menu.add("Gyro Calib", &gyroSwing);
    //menu.sdd("Reload Config", &loadConfig);
    menu.add("Backlight", &setBacklight);
    menu.add("Reverse", &reverseScreen);
    menu.add("Reset", &resetMe);

    fputs("Starting main timer...\n", stdout);

    timer.start();
    timer.reset();

    int thisUpdate = timer.read_ms();    
    int nextUpdate = thisUpdate;
    char cmd;
    bool printMenu = true;
    bool printLCDMenu = true;

    fputs("Timer done, enter loop...\n", stdout);

    while (1) {

        /*
        if (timer.read_ms() > hdgUpdate) {
            fprintf(stdout, "He=%.3f %.5f\n", kfGetX(0), kfGetX(1));
            hdgUpdate = timer.read_ms() + 100;
        }*/

        if ((thisUpdate = timer.read_ms()) > nextUpdate) {
            // Pulling out current state so we get the most current
            SystemState *s = fifo_first();
            // Now populate in the current GPS data
            s->gpsHDOP = sensors.gps.hdop();
            s->gpsSats = sensors.gps.sat_count();
            display.update(s);
            nextUpdate = thisUpdate + 2000;
            // TODO 3 move this statistic into display class
            //fprintf(stdout, "update time: %d\n", getUpdateTime());
        }
        
        if (keypad.pressed) {
            keypad.pressed = false;
            printLCDMenu = true;
            switch (keypad.which) {
                case NEXT_BUTTON:
                    menu.next();
                    break;
                case PREV_BUTTON:
                    menu.prev();
                    break;
                case SELECT_BUTTON:
                    display.select(menu.getItemName());
                    menu.select();
                    printMenu = true;
                    break;
                default:
                    printLCDMenu = false;
                    break;
            }//switch  
            keypad.pressed = false;
        }// if (keypad.pressed)

            
        if (printLCDMenu) {
            display.menu( menu.getItemName() );
            display.status("Ready.");
            display.redraw();
            printLCDMenu = false;
        }
        
        // TODO 2 move to UI area
        if (printMenu) {
            fprintf(stdout, "\n==============\nData Bus Menu\n==============\n");
            fputs("0) Autonomous mode\n", stdout);
            fputs("1) Bridge serial to GPS\n", stdout);
            fputs("2) Calibrate compass\n", stdout);
            fputs("3) Swing compass\n", stdout);
            fputs("4) Gyro calibrate\n", stdout);
            fputs("5) Instrument check\n", stdout);
            fputs("6) Display AHRS\n", stdout);
            fputs("7) Mavlink mode\n", stdout);
            fputs("8) Telemetry mode\n", stdout);
            fputs("9) Shell\n", stdout);
            fputs("R) Reset\n", stdout);
            fputs("\nSelect from the above: ", stdout);
            fflush(stdout);
            printMenu = false;
        }

        // Basic functional architecture
        // SENSORS -> FILTERS -> AHRS -> POSITION -> NAVIGATION -> CONTROL | INPUT/OUTPUT | LOGGING
        // SENSORS (for now) are polled out of AHRS via interrupt every 10ms
        //
        // no FILTERing in place right now
        // if we filter too heavily we get lag. At 30mph = 14m/s a sensor lag
        // of only 10ms means the estimate is 140cm behind the robot
        //
        // POSITION and NAVIGATION should probably always be running
        // log file can have different entry type per module, to be demultiplexed on the PC
        //
        // Autonomous mode engages CONTROL outputs
        //
        // I/O mode could be one of: MAVlink, serial bridge (gps), sensor check, shell, log to serial
        // Or maybe shell should be the main control for different output modes
        //
        // LOGGING can be turned on or off, probably best to start with it engaged
        // and then disable from user panel or when navigation is ended

        if (pc.readable()) {
            cmd = fgetc(stdin);
            fputc(cmd, stdout);
            fputc('\n', stdout);
            printMenu = true;
            printLCDMenu = true;
            
            switch (cmd) {
                case 'R' :
                    resetMe();
                    break;
                case '0' :
                    display.select(menu.getItemName(0));
                    autonomousMode();
                    break;
                case '1' :
                    display.select("Serial bridge");
                    display.status("Standby.");
                    sensors.gps.enableVerbose();
                    serialBridge( *(sensors.gps.getSerial()) );
                    sensors.gps.disableVerbose();
                    break;
                case '2' :
                    display.select(menu.getItemName(1));
                    compassCalibrate();
                    break;
                case '3' :
                    display.select(menu.getItemName(2));
                    compassSwing();
                    break;
                case '4' :
                    display.select(menu.getItemName(2));
                    gyroSwing();
                    break;
                case '5' :
                    display.select("Instruments");
                    display.status("Standby.");
                    displayData(INSTRUMENT_CHECK);
                    break;
                case '6' :
                    display.select("AHRS Visual'n");
                    display.status("Standby.");
                    displayData(AHRS_VISUALIZATION);
                    break;
                case '7' :
                    display.select("Mavlink mode");
                    display.status("Standby.");
                    mavlinkMode();
                    break;
                case '8' :
                    display.select("Telemetry mode");
                    display.status("Standby.");
                    telemetryMode();
                    break;
                case '9' :
                    display.select("Shell");
                    display.status("Standby.");
                    shell(0);
                    break;
                /*
                case 'A' :
                    display.select("Serial bridge 2");
                    display.status("Standby.");
                    //gps2.enableVerbose();
                    //serialBridge( *(gps2.getSerial()) );
                    //gps2.disableVerbose();
                    break;
                */
                default :
                    break;
            } // switch        

		} // if (pc.readable())

        wait(0.1);

    } // while

}



///////////////////////////////////////////////////////////////////////////////////////////////////////
// INITIALIZATION ROUTINES
///////////////////////////////////////////////////////////////////////////////////////////////////////
    
void initFlasher()
{ 
    // Set up flasher schedule; 3 flashes every 80ms
    // for 80ms total, with a 9x80ms period
    blink.max(9);
    blink.scale(80);
    blink.mode(Schedule::repeat);
    blink.set(0, 1);  blink.set(2, 1);  blink.set(4, 1);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// OPERATIONAL MODE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////

int autonomousMode()
{
    bool goGoGo = false;                    // signal to start moving
    bool navDone;                      // signal that we're done navigating
    //extern int tSensor, tGPS, tAHRS, tLog;

    sensors.gps.reset_available();

    // TODO: 3 move to main?
    // Navigation

    goGoGo = false;
    navDone = false;
    keypad.pressed = false;
    //bool started = false;  // flag to indicate robot has exceeded min speed.
    
    if (initLogfile()) logStatus = 1; // Open the log file in sprintf format string style; numbers go in %d
    wait(0.2);

    sensors.gps.disableVerbose();
    sensors.gps.enable();
    //gps2.enable();

    fprintf(stdout, "Press select button to start.\n");
    display.status("Select starts.");
    wait(1.0);
    
    timer.reset();
    timer.start();
    wait(0.1);
    
    // Tell the navigation / position estimation stuff to reset to starting waypoint
    // Disable 05/27/2013 to try and fix initial heading estimate
    //restartNav();

    // Main loop
    //
    while(navDone == false) {
        //////////////////////////////////////////////////////////////////////////////
        // USER INPUT
        //////////////////////////////////////////////////////////////////////////////

        // Button state machine
        // if we've not started going, button starts us
        // if we have started going, button stops us
        // but only if we've released it first
        //
        // set throttle only if goGoGo set
        if (goGoGo) {
            // TODO: 1 Add additional condition of travel for N meters before
            // the HALT button is armed
            
            if (keypad.pressed == true) { // && started
                fprintf(stdout, ">>>>>>>>>>>>>>>>>>>>>>> HALT\n");
                display.status("HALT.");
                navDone = true;
                goGoGo = false;
                keypad.pressed = false;
                endRun();
            }
        } else {
            if (keypad.pressed == true) {
                fprintf(stdout, ">>>>>>>>>>>>>>>>>>>>>>> GO GO GO\n");
                display.status("GO GO GO!");
                goGoGo = true;
                keypad.pressed = false;
                beginRun();
            }
        }        

        // Are we at the last waypoint?
        // 
        if (fifo_first()->nextWaypoint == config.wptCount) {
            fprintf(stdout, "Arrived at final destination.\n");
            display.status("Arrived at end.");
            navDone = true;
            endRun();
        }

        //////////////////////////////////////////////////////////////////////////////
        // LOGGING
        //////////////////////////////////////////////////////////////////////////////
        // sensor reads are happening in the schedHandler();
        // Are there more items to come out of the log fifo?
        // Since this could take anywhere from a few hundred usec to
        // 150ms, we run it opportunistically and use a buffer. That way
        // the sensor updates, calculation, and control can continue to happen
        if (fifo_available()) {
            logStatus = !logStatus;         // log indicator LED
            logData( fifo_pull() );         // log state data to file
            logStatus = !logStatus;         // log indicator LED
        }

    } // while
    closeLogfile();
    wait(2.0);
    logStatus = 0;
    display.status("Completed. Saved.");
    wait(2.0);

    updaterStatus = 0;
    gpsStatus = 0;
    //confStatus = 0;
    //flasher = 0;

    sensors.gps.disableVerbose();

    return 0;
} // autonomousMode


///////////////////////////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////


int compassCalibrate()
{
    bool done=false;
    int m[3];
    FILE *fp;
    
    fprintf(stdout, "Entering compass calibration in 2 seconds.\nLaunch _3DScatter Processing app now... type e to exit\n");
    display.status("Starting...");

    fp = openlog("cal");

    wait(2);
    display.status("Select exits");
    timer.reset();
    timer.start();
    while (!done) {
    
        if (keypad.pressed) {
            keypad.pressed = false;
            done = true;
        }
        
        while (pc.readable()) {
            if (fgetc(stdin) == 'e') {
                done = true;
                break;
            }
        }
        int millis = timer.read_ms();
        if ((millis % 100) == 0) {
            sensors.getRawMag(m);

            // Correction
            // Let's see how our ellipsoid looks after scaling and offset            
            /*
            float mag[3];
            mag[0] = ((float) m[0] - M_OFFSET_X) * 0.5 / M_SCALE_X;
            mag[1] = ((float) m[1] - M_OFFSET_Y) * 0.5 / M_SCALE_Y;
            mag[2] = ((float) m[2] - M_OFFSET_Z) * 0.5 / M_SCALE_Z;  
            */
            
            bool skipIt = false;
            for (int i=0; i < 3; i++) {
                if (abs(m[i]) > 1024) skipIt = true;
            }
            if (!skipIt) {
                fprintf(stdout, "%c%d %d %d \r\n", 0xDE, m[0], m[1], m[2]);
                fprintf(fp, "%d, %d, %d\n", m[0], m[1], m[2]);
            }
        }
    }
    if (fp) {
        fclose(fp);
        display.status("Done. Saved.");
        wait(2);
    }

    return 0;
}

// Gather gyro data using turntable equipped with dual channel
// encoder. Use onboard wheel encoder system. Left channel
// is the index (0 degree) mark, while the right channel
// is the incremental encoder.  Can then compare gyro integrated
// heading with machine-reported heading
//
// Note: some of this code is identical to the compassSwing() code.
//
int gyroSwing()
{
    FILE *fp;

    // Timing is pretty critical so just in case, disable serial processing from GPS
    sensors.gps.disable();

    fprintf(stdout, "Entering gyro swing...\n");
    display.status("Starting...");
    wait(2);
    fp = openlog("gy");
    wait(2);
    display.status("Begin. Select exits.");

    fprintf(stdout, "Begin clockwise rotation, varying rpm... press select to exit\n");

    timer.reset();
    timer.start();

    sensors.rightTotal = 0; // reset total
    sensors._right.read();  // easiest way to reset the heading counter
    
    while (1) {
        if (keypad.pressed) {
            keypad.pressed = false;
            break;
        }

        // Print out data
        // fprintf(stdout, "%d,%d,%d,%d,%d\n", timer.read_ms(), heading, sensors.g[0], sensors.g[1], sensors.g[2]);
        // sensors.rightTotal gives us each tick of the machine, multiply by 2 for cumulative heading, which is easiest
        // to compare with cumulative integration of gyro (rather than dealing with 0-360 degree range and modulus and whatnot
        if (fp) fprintf(fp, "%d,%d,%d,%d,%d,%d\n", timer.read_ms(), 2*sensors.rightTotal, sensors.g[0], sensors.g[1], sensors.g[2], sensors.gTemp);
        wait(0.200);
    }    
    if (fp) {
        fclose(fp);
        display.status("Done. Saved.");
        fprintf(stdout, "Data collection complete.\n");
        wait(2);
    }
    
    keypad.pressed = false;

    return 0;
}


// Swing compass using turntable equipped with dual channel
// encoder. Use onboard wheel encoder system. Left channel
// is the index (0 degree) mark, while the right channel
// is the incremental encoder.
//
// Note: much of this code is identical to the gyroSwing() code.
//
int compassSwing()
{
    int revolutions=5;
    int heading=0;
    int leftCount = 0;
    FILE *fp;
    // left is index track
    // right is encoder track

    fprintf(stdout, "Entering compass swing...\n");
    display.status("Starting...");
    wait(2);
    fp = openlog("sw");
    wait(2);
    display.status("Ok. Begin.");

    fprintf(stdout, "Begin clockwise rotation... exit after %d revolutions\n", revolutions);

    timer.reset();
    timer.start();

    // wait for index to change
    while ((leftCount += sensors._left.read()) < 2) {
        if (keypad.pressed) {
            keypad.pressed = false;
            break;    
        }
    }
    fprintf(stdout, ">>>> Index detected. Starting data collection\n");
    leftCount = 0;
    // TODO 3 how to parameterize status?
    lcd.pos(0,1);
    lcd.printf("%1d %-14s", revolutions, "revs left");

    sensors._right.read(); // easiest way to reset the heading counter
    
    while (revolutions > 0) {
        int encoder;

        if (keypad.pressed) {
            keypad.pressed = false;
            break;
        }
               
        // wait for state change
        while ((encoder = sensors._right.read()) == 0) {
            if (keypad.pressed) {
                keypad.pressed = false;
                break;
            }
        }
        heading += 2*encoder;                          // encoder has resolution of 2 degrees
        if (heading >= 360) heading -= 360;
                
        // when index is 1, reset the heading and decrement revolution counter
        // make sure we don't detect the index mark until after the first several
        // encoder pulses.  Index is active low
        if ((leftCount += sensors._left.read()) > 1) {
            // check for error in heading?
            leftCount = 0;
            revolutions--;
            fprintf(stdout, ">>>>> %d left\n", revolutions); // we sense the rising and falling of the index so /2
            lcd.pos(0,1);
            lcd.printf("%1d %-14s", revolutions, "revs left");
        }
        
        float heading2d = 180 * atan2((float) sensors.mag[1], (float) sensors.mag[0]) / PI;
        // Print out data
        //getRawMag(m);
        fprintf(stdout, "%d %.4f\n", heading, heading2d);

//        int t1=t.read_us();
        if (fp) fprintf(fp, "%d, %d, %.2f, %.4f, %.4f, %.4f\n", 
                            timer.read_ms(), heading, heading2d, sensors.mag[0], sensors.mag[1], sensors.mag[2]);
//        int t2=t.read_us();
//        fprintf(stdout, "dt=%d\n", t2-t1);
    }    
    if (fp) {
        fclose(fp);
        display.status("Done. Saved.");
        fprintf(stdout, "Data collection complete.\n");
        wait(2);
    }
    
    keypad.pressed = false;
        
    return 0;
}

void servoCalibrate() 
{
}

void bridgeRecv()
{
    while (dev && dev->readable()) {
        pc.putc(dev->getc());
    }
}

void serialBridge(Serial &serial)
{
    char x;
    int count = 0;
    bool done=false;

    fprintf(stdout, "\nEntering serial bridge in 2 seconds, +++ to escape\n\n");
    sensors.gps.enableVerbose();
    wait(2.0);
    //dev = &gps;
    sensors.gps.disable();
    serial.baud(38400);
    while (!done) {
        if (pc.readable()) {
            x = pc.getc();
            serial.putc(x);
            // escape sequence
            if (x == '+') {
                if (++count >= 3) done=true;
            } else {
                count = 0;
            }
        }
        if (serial.readable()) {
            pc.putc(serial.getc());
        }
    }
}

/* to be called from panel menu
 */
int instrumentCheck(void) {
    displayData(INSTRUMENT_CHECK);
    return 0;
}

/* Display data
 * mode determines the type of data and format
 * INSTRUMENT_CHECK   : display readings of various instruments
 * AHRS_VISUALIZATION : display data for use by AHRS python visualization script
 */
 
void displayData(const int mode)
{
    bool done = false;

    lcd.clear();

    // Init GPS
    sensors.gps.disableVerbose();
    sensors.gps.enable();
    sensors.gps.reset_available();    

    // Init 2nd GPS
    //gps2.enable();
    //gps2.reset_available();

    keypad.pressed = false;  
    
    timer.reset();
    timer.start();
      
    fprintf(stdout, "press e to exit\n");
    while (!done) {
        int millis = timer.read_ms();

        if (keypad.pressed) {
            keypad.pressed = false;
            done=true;
        }
        
        while (pc.readable()) {
            if (pc.getc(stdin) == 'e') {
                done = true;
                break;
            }
        }

/*        
        if (mode == AHRS_VISUALIZATION && (millis % 100) == 0) {

            fprintf(stdout, "!ANG:%.1f,%.1f,%.1f\r\n", ToDeg(ahrs.roll), ToDeg(ahrs.pitch), ToDeg(ahrs.yaw));

        } else if (mode == INSTRUMENT_CHECK) {
 */
        
            if ((millis % 1000) == 0) {
            	SystemState *s = fifo_first();

                fprintf(stdout, "update() time = %.3f msec\n", getUpdateTime() / 1000.0);
                fprintf(stdout, "Rangers: L=%.2f R=%.2f C=%.2f", sensors.leftRanger, sensors.rightRanger, sensors.centerRanger);
                fprintf(stdout, "\n");
                //fprintf(stdout, "ahrs.MAG_Heading=%4.1f\n",  ahrs.MAG_Heading*180/PI);
                //fprintf(stdout, "raw m=(%d, %d, %d)\n", sensors.m[0], sensors.m[1], sensors.m[2]);
                //fprintf(stdout, "m=(%2.3f, %2.3f, %2.3f) %2.3f\n", sensors.mag[0], sensors.mag[1], sensors.mag[2],
                //        sqrt(sensors.mag[0]*sensors.mag[0] + sensors.mag[1]*sensors.mag[1] + sensors.mag[2]*sensors.mag[2] ));
                fprintf(stdout, "g=(%4d, %4d, %4d) %d\n", sensors.g[0], sensors.g[1], sensors.g[2], sensors.gTemp);
                fprintf(stdout, "gc=(%.1f, %.1f, %.1f)\n", sensors.gyro[0], sensors.gyro[1], sensors.gyro[2]);
                fprintf(stdout, "a=(%5d, %5d, %5d)\n", sensors.a[0], sensors.a[1], sensors.a[2]);
                fprintf(stdout, "estHdg=%.2f lagHdg=%.2f\n", s->estHeading, s->estLagHeading);
                //fprintf(stdout, "roll=%.2f pitch=%.2f yaw=%.2f\n", ToDeg(ahrs.roll), ToDeg(ahrs.pitch), ToDeg(ahrs.yaw));
                fprintf(stdout, "speed: left=%.3f  right=%.3f\n", sensors.lrEncSpeed, sensors.rrEncSpeed);
                fprintf(stdout, "gps=(%.6f, %.6f, h=%.1f, s=%.1f, hdop=%.1f, sat=%d)\n",
                    sensors.gps.latitude(), sensors.gps.longitude(), sensors.gps.heading_deg(), 
                    sensors.gps.speed_mps(), sensors.gps.hdop(), sensors.gps.sat_count());
                fprintf(stdout, "brg=%6.2f d=%8.4f sa=%6.2f\n", s->bearing, s->distance, s->steerAngle);
                /*
                fprintf(stdout, "gps2=(%.6f, %.6f, %.1f, %.1f, %.1f, %d) %02x\n", 
                    gps2.latitude(), gps2.longitude(), gps2.heading_deg(), gps2.speed_mps(), gps2.hdop(), gps2.sat_count(),
                    (unsigned char) gps2.getAvailable() );
                */
                fprintf(stdout, "v=%.2f  a=%.3f\n", sensors.voltage, sensors.current);
                fprintf(stdout, "\n");
                
            }

            if ((millis % 3000) == 0) {

                lcd.pos(0,1);
                //lcd.printf("H=%4.1f   ", ahrs.MAG_Heading*180/PI);
                //wait(0.1);
                lcd.pos(0,2);
                lcd.printf("G=%4.1f,%4.1f,%4.1f    ", sensors.gyro[0], sensors.gyro[1], sensors.gyro[2]);
                wait(0.1);
                lcd.pos(0,3);
                lcd.printf("La=%11.6f HD=%1.1f  ", sensors.gps.latitude(), sensors.gps.hdop());
                wait(0.1);
                lcd.pos(0,4);
                lcd.printf("Lo=%11.6f Sat=%-2d  ", sensors.gps.longitude(), sensors.gps.sat_count());
                wait(0.1);
                lcd.pos(0,5);
                lcd.printf("V=%5.2f A=%5.3f  ", sensors.voltage, sensors.current);
                
            }
        //}
    
    } // while !done
    // clear input buffer
    while (pc.readable()) pc.getc();
    lcd.clear();
    updaterStatus = 0;
    gpsStatus = 0;
}


void telemetryMode() {
	RawSerial pc(USBTX, USBRX);
	bool done=false;
	static int skip = 0;

	pc.baud(115200);

	// FIXME - escape from telemetry mode
	pc.puts("Entering telemetry mode; press e to exit\n\n");

    timer.reset();
    timer.start();

    beginRun();

    while (!done) {

        if (keypad.pressed) {
            keypad.pressed = false;
            done=true;
        }

        while (pc.readable()) {
            if (pc.getc() == 'e') {
                done = true;
                break;
            }
        }

        unsigned int millis = timer.read_ms();

        pc.printf("fifo in:%d out:%d\n", fifo_getInState(), fifo_getOutState());

		SystemState *s = fifo_pull();

		if (s && ++skip > 5) {
			skip = 0;

			confStatus = 1;
			pc.printf("^%u, ", millis);
			pc.printf("%.2f, %.2f, ", s->voltage, s->current);
			pc.printf("%.2f, %.7f, %.7f, %.1f, %d, ",
					s->estHeading,
					s->gpsLatitude, s->gpsLongitude,
					s->gpsHDOP, s->gpsSats );
			pc.printf("%.1f, ", (s->lrEncSpeed + s->rrEncSpeed)/2.0);
			pc.printf("%.2f, %.5f, %.2f\n", s->bearing, s->distance, s->steerAngle);
			confStatus = 0;
		}
    }
    endRun();

	return;
}

// TODO: 3 move Mavlink into main (non-interrupt) loop along with logging
// possibly also buffered if necessary

void mavlinkMode() {
#if 0
    uint8_t system_type = MAV_FIXED_WING;
    uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
    //int count = 0;
    bool done = false;
    
    mavlink_system.sysid = 100; // System ID, 1-255
    mavlink_system.compid = 200; // Component/Subsystem ID, 1-255

    //mavlink_attitude_t mav_attitude;
    //mavlink_sys_status_t mav_stat;
    mavlink_vfr_hud_t mav_hud;
 
    //mav_stat.mode = MAV_MODE_MANUAL;
    //mav_stat.status = MAV_STATE_STANDBY;
    //mav_stat.vbat = 8400;
    //mav_stat.battery_remaining = 1000;

    mav_hud.airspeed = 0.0;
    mav_hud.groundspeed = 0.0;
    mav_hud.throttle = 0;

    fputs("Entering MAVlink mode; reset the MCU to exit\n\n", stdout);

    wait(5.0);

    //gps.gsvMessage(true);
    //gps.gsaMessage(true);
    //gps.serial.attach(gpsRecv, Serial::RxIrq);
    
    timer.start();
    timer.reset();
    
    while (done == false) {

        if (keypad.pressed == true) { // && started
            keypad.pressed = false;
            done = true;
        }

        int millis = timer.read_ms();
      
        if ((millis % 1000) == 0) {
            SystemState *s = fifo_first();
        /*
        s.millis,
        s.current, s.voltage,
        s.g[0], s.g[1], s.g[2],
        s.gTemp,
        s.a[0], s.a[1], s.a[2],
        s.m[0], s.m[1], s.m[2],
        s.gHeading, //s.cHeading,
        //s.roll, s.pitch, s.yaw,
        s.gpsLatitude, s.gpsLongitude, s.gpsCourse, s.gpsSpeed*0.44704, s.gpsHDOP, s.gpsSats, // convert gps speed to m/s
        s.lrEncDistance, s.rrEncDistance, s.lrEncSpeed, s.rrEncSpeed, s.encHeading,
        s.estHeading, s.estLatitude, s.estLongitude,
        // s.estNorthing, s.estEasting, 
        s.estX, s.estY,
        s.nextWaypoint, s.bearing, s.distance, s.gbias, s.errAngle,
        s.leftRanger, s.rightRanger, s.centerRanger,
        s.crossTrackErr
        */

            float groundspeed = (s->lrEncSpeed + s->rrEncSpeed)/2.0;
            //mav_hud.groundspeed *= 2.237; // convert to mph
            //mav_hud.heading = compassHeading();

            mav_hud.heading = 0.0; //ahrs.parser.yaw;
            
            mavlink_msg_attitude_send(MAVLINK_COMM_0, millis*1000, 
                0.0, //ToDeg(ahrs.roll),
                0.0, //ToDeg(ahrs.pitch),
                s->estHeading,
                0.0, // rollspeed
                0.0, // pitchspeed
                0.0  // yawspeed
            );


            mavlink_msg_vfr_hud_send(MAVLINK_COMM_0, 
                    groundspeed, 
                    groundspeed, 
                    s->estHeading,
                    mav_hud.throttle, 
                    0.0, // altitude
                    0.0  // climb
            );

            mavlink_msg_heartbeat_send(MAVLINK_COMM_0, system_type, autopilot_type);
            mavlink_msg_sys_status_send(MAVLINK_COMM_0,
                    MAV_MODE_MANUAL,
                    MAV_NAV_GROUNDED,
                    MAV_STATE_STANDBY,
                    0.0, // load
                    (uint16_t) (sensors.voltage * 1000),
                    1000, // TODO: 3 fix batt remaining
                    0 // packet drop
            );
            
            
            mavlink_msg_gps_raw_send(MAVLINK_COMM_0, millis*1000, 3, 
                sensors.gps.latitude(), 
                sensors.gps.longitude(), 
                0.0, // altitude
                sensors.gps.hdop()*100.0, 
                0.0, // VDOP
                groundspeed, 
                s->estHeading
            );
                
            mavlink_msg_gps_status_send(MAVLINK_COMM_0, sensors.gps.sat_count(), 0, 0, 0, 0, 0);

            wait(0.001);
        } // millis % 1000

        /*
        if (gps.nmea.rmc_ready() &&sensors.gps.nmea.gga_ready()) {
            char gpsdate[32], gpstime[32];

           sensors.gps.process(gps_here, gpsdate, gpstime);
            gpsStatus = (gps.hdop > 0.0 && sensors.gps.hdop < 3.0) ? 1 : 0;

            mavlink_msg_gps_raw_send(MAVLINK_COMM_0, millis*1000, 3, 
                gps_here.latitude(), 
                gps_here.longitude(), 
                0.0, // altitude
                sensors.gps.nmea.f_hdop()*100.0, 
                0.0, // VDOP
                mav_hud.groundspeed, 
                mav_hud.heading
            );
                
            mavlink_msg_gps_status_send(MAVLINK_COMM_0, sensors.gps.nmea.sat_count(), 0, 0, 0, 0, 0);

            sensors.gps.nmea.reset_ready();
                
        } //gps

        //mavlink_msg_attitude_send(MAVLINK_COMM_0, millis*1000, mav_attitude.roll, mav_attitude.pitch, mav_attitude.yaw, 0.0, 0.0, 0.0);
        //mavlink_msg_sys_status_send(MAVLINK_COMM_0, mav_stat.mode, mav_stat.nav_mode, mav_stat.status, mav_stat.load,
        //                            mav_stat.vbat, mav_stat.battery_remaining, 0);

        */

    }

    //gps.serial.attach(NULL, Serial::RxIrq);
    //gps.gsvMessage(false);
    //gps.gsaMessage(false);
    
    fprintf(stdout, "\n");
    
    return;
#endif
}

// TODO 2 move to display
int setBacklight(void) {
    Menu bmenu;
    bool done = false;
    bool printUpdate = false;
    static int backlight=100;
    
    display.select(">> Backlight");

    while (!done) {
        if (keypad.pressed) {
            keypad.pressed = false;
            printUpdate = true;
            switch (keypad.which) {
                case NEXT_BUTTON:
                    backlight+=5;
                    if (backlight > 100) backlight = 100;
                    lcd.backlight(backlight);
                    break;
                case PREV_BUTTON:
                    backlight-=5;
                    if (backlight < 0) backlight = 0;
                    lcd.backlight(backlight);
                    break;
                case SELECT_BUTTON:
                    done = true;
                    break;    
            }
        }
        if (printUpdate) {
            printUpdate = false;
            lcd.pos(0,1);
            lcd.printf("%3d%%%-16s", backlight, "");
        }
    }
    
    return 0;
}

int reverseScreen(void) {
    lcd.reverseMode();
    
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// ADC CONVERSION FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////

// returns distance in m for Sharp GP2YOA710K0F
// to get m and b, I wrote down volt vs. dist by eyeballin the
// datasheet chart plot. Then used Excel to do linear regression
//
float irDistance(const unsigned int adc)
{
    float b = 1.0934; // Intercept from Excel
    float m = 1.4088; // Slope from Excel

    return m / (((float) adc) * 4.95/4096 - b);
}
