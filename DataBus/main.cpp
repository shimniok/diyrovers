/** Code for "Data Bus" UGV entry for Sparkfun AVC 2014
 *  http://www.bot-thoughts.com/
 */

///////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////////////////////////////

#include "mbed.h"
#include <math.h>
#include <stdint.h>
#include "devices.h"
#include "globals.h"
#include "Filesystem.h"
#include "SerialMux.h"
#include "Config.h"
#include "Buttons.h"
#include "Display.h"
#include "Menu.h"
#include "GPSStatus.h"
#include "logging.h"
#include "Telemetry.h"
#include "SystemState.h"
#include "shell.h"
#include "Steering.h"
#include "Sensors.h"
#include "kalman.h"
#include "Ublox6.h"
#include "PinDetect.h" // TODO 4 this should be broken into .h, .cpp
#include "IncrementalEncoder.h"
#include "Steering.h"
#include "Schedule.h"
#include "GeoPosition.h"
#include "Mapping.h"
#include "SimpleFilter.h"
#include "Beep.h"
#include "util.h"
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

// COMM
Serial pc(USBTX, USBRX);                // PC usb communications
SerialMux mux(&pc);						// Multiplexed output
SerialGraphicLCD lcd(LCDTX, LCDRX, SD_FW);  // Graphic LCD with summoningdark firmware
Serial tel(TELEMTX, TELEMRX);			// UART for telemetry
Telemetry telem(tel);					// Setup telemetry system

// SENSORS
Sensors sensors;                        // Abstraction of sensor drivers
//DCM ahrs;                             // ArduPilot/MatrixPilot AHRS
Serial *dev;                            // For use with bridge

// MISC
FILE *camlog;                           // Camera log
Filesystem fs;							// Set up filesystems
Config config;							// Configuration utility

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
void displayData(const int mode);
int autonomousMode(void);
void telemetryMode(void);
void servoCalibrate(void);
void serialBridge(Serial &gps);
int instrumentCheck(void);
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

    // Let's try setting priorities...
    //NVIC_SetPriority(DMA_IRQn, 0);
    NVIC_SetPriority(TIMER3_IRQn, 2);   // updater running off Ticker, must be highest priority!!
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

    display.init();
    display.status("Data Bus 2014");

    // Send data back to the PC
    pc.baud(115200);
    fputs("Data Bus 2014\n", stdout);
    fflush(stdin);

    fputs("Initializing...\n", stdout);
    display.status("Initializing");
    
    // Initialize status LEDs
    updaterStatus = 0;
    gpsStatus = 0;
    logStatus = 0;
    confStatus = 0;

    if (!fifo_init()) {
    	error("\n\n%% Error initializing SystemState fifo %%\n");
    }

    fputs("Loading configuration...\n", stdout);
    display.status("Load config");
    if (config.load("/etc/config.txt"))                          // Load various configurable parameters, e.g., waypoints, declination, etc.
        confStatus = 1;

    initThrottle();

    //pc.printf("Declination: %.1f\n", config.declination);
    pc.puts("Speed: escZero=");
    pc.puts(cvftos(config.escZero, 3));
    pc.puts(" escMin=");
    pc.puts(cvftos(config.escMin, 3));
    pc.puts(" escMax=");
    pc.puts(cvftos(config.escMax, 3));
    pc.puts(" top=");
    pc.puts(cvftos(config.topSpeed, 1));
    pc.puts(" turn=");
    pc.puts(cvftos(config.turnSpeed, 1));
    pc.puts(" Kp=");
    pc.puts(cvftos(config.speedKp, 4));
    pc.puts(" Ki=");
    pc.puts(cvftos(config.speedKi, 4));
    pc.puts(" Kd=");
    pc.puts(cvftos(config.speedKd, 4));
    pc.puts("\n");

    pc.puts("Steering: steerZero=");
    pc.puts(cvftos(config.steerZero, 2));
    pc.puts(" steerScale=");
    pc.puts(cvftos(config.steerScale, 1));
    pc.puts("\n");
    steering.setScale(config.steerScale);

    // Convert lat/lon waypoints to cartesian
    mapper.init(config.wptCount, config.wpt);
    for (unsigned int w = 0; w < MAXWPT && w < config.wptCount; w++) {
        mapper.geoToCart(config.wpt[w], &(config.cwpt[w]));
        pc.puts("Waypoint #");
        pc.puts(cvntos(w));
        pc.puts(" (");
        pc.puts(cvftos(config.cwpt[w].x, 4));
        pc.puts(", ");
        pc.puts(cvftos(config.cwpt[w].y, 4));
        pc.puts(") lat: ");
        pc.puts(cvftos(config.wpt[w].latitude(), 6));
        pc.puts(" lon: ");
        pc.puts(cvftos(config.wpt[w].longitude(), 6));
        pc.puts(", topspeed: ");
        pc.puts(cvftos(config.topSpeed + config.wptTopSpeedAdj[w], 1));
        pc.puts(", turnspeed: ");
        pc.puts(cvftos(config.turnSpeed + config.wptTurnSpeedAdj[w], 1));
        pc.puts("\n");
    }

    // TODO 3 print mag and gyro calibrations

    // TODO 3 remove GPS configuration, all config will be in object itself I think

    display.status("Vehicle config      ");
    pc.puts("Wheelbase: ");
    pc.puts(cvftos(config.wheelbase, 3));
    pc.puts("\n");
    pc.puts("Track Width: ");
    pc.puts(cvftos(config.track, 3));
    pc.puts("\n");
    steering.setWheelbase(config.wheelbase);
    steering.setTrack(config.track);

    display.status("Encoder config      ");
    pc.puts("Tire Circumference: ");
    pc.puts(cvftos(config.tireCirc, 5));
    pc.puts("\n");
    pc.puts("Ticks per revolution: ");
    pc.puts(cvftos(config.encStripes, 5));
    pc.puts("\n");
    sensors.configureEncoders(config.tireCirc, config.encStripes);

    display.status("Nav configuration   ");
    pc.puts("Intercept distance: ");
    pc.puts(cvftos(config.intercept, 1));
    pc.puts("\n");
    steering.setIntercept(config.intercept);
    pc.puts("Waypoint distance: ");
    pc.puts(cvftos(config.waypointDist, 1));
    pc.puts("\n");
	pc.puts("Brake distance: ");
    pc.puts(cvftos(config.brakeDist, 1));
    pc.puts("\n");
    pc.puts("Min turn radius: ");
	pc.puts(cvftos(config.minRadius, 3));
    pc.puts("\n");

    display.status("Gyro config         ");
    pc.puts("\n");
    pc.puts("Gyro scale: ");
    pc.puts(cvftos(config.gyroScale, 5));
    pc.puts("\n");
    sensors.setGyroScale(config.gyroScale);

    pc.puts("Calculating offsets...\n");
    display.status("Offset calculation  ");
    wait(0.2);
    // TODO 3 Really need to give the gyro more time to settle
    sensors.gps.disable();
    // TODO 2 sensors.Calculate_Offsets();

    pc.puts("Starting GPS...\n");
    display.status("Start GPS           "); // TODO 3: would be nice not to have to pad at this level
    wait(0.2);
    sensors.gps.setUpdateRate(10);
    sensors.gps.enable();

    pc.puts("Starting Scheduler...\n");
    display.status("Start scheduler     ");
    wait(0.2);
    // Startup sensor/AHRS ticker; update every UPDATE_PERIOD
    restartNav();
    startUpdater();

    pc.puts("Starting keypad...\n");

    keypad.init();
    
    pc.puts("Adding menu items...\n");

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

    pc.puts("Starting main timer...\n");

    timer.start();
    timer.reset();

    int thisUpdate = timer.read_ms();    
    int nextDisplayUpdate = thisUpdate;
    int nextWaypointUpdate = thisUpdate;
    char cmd;
    bool printMenu = true;
    bool printLCDMenu = true;

    pc.puts("Timer done, enter loop...\n");

    while (1) {

        thisUpdate = timer.read_ms();
        if (thisUpdate > nextDisplayUpdate) {
            // Pulling out current state so we get the most current
            SystemState *s = fifo_first();
            // TODO 3 fix this so gps is already in state
            // Now populate in the current GPS data
            s->gpsHDOP = sensors.gps.hdop();
            s->gpsSats = sensors.gps.sat_count();

            telem.sendPacket(s);
            display.update(s);
            nextDisplayUpdate = thisUpdate + 200;
        }

        // every so often, send the currently configured waypoints
        if (thisUpdate > nextWaypointUpdate) {
        	telem.sendPacket(config.cwpt, config.wptCount);
        	nextWaypointUpdate = thisUpdate + 10000;
        	// TODO 2: make this a request/response, Telemetry has to receive packets, decode, etc.
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
        
        // TODO 3 move to UI area
        if (printMenu) {
            fputs("\n==============\nData Bus Menu\n==============\n", stdout);
            fputs("0) Autonomous mode\n", stdout);
            fputs("1) Bridge serial to GPS\n", stdout);
            fputs("2) Calibrate compass\n", stdout);
            fputs("3) Swing compass\n", stdout);
            fputs("4) Gyro calibrate\n", stdout);
            //fputs("5) Instrument check\n", stdout);
            fputs("5) Telemetry mode\n", stdout);
            fputs("6) Shell\n", stdout);
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
        // I/O mode could be one of: telemetry, serial bridge (gps), sensor check, shell, log to serial
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
				/*
                case '5' :
                    display.select("Instruments");
                    display.status("Standby.");
                    displayData(INSTRUMENT_CHECK);
                    break;
                */
                case '5' :
                	display.select("Telemetry");
                	display.status("Standby.");
                	telemetryMode();
                	break;
                case '6' :
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
	bool goGoGo = false;               	// signal to start moving
    bool navDone;                      	// signal that we're done navigating
    int nextTelemUpdate;				// keeps track of teletry update periods
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

    fputs("Press select button to start.\n", stdout);
    display.status("Select starts.");
    wait(1.0);
    
    timer.reset();
    timer.start();
    nextTelemUpdate = timer.read_ms();
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
            // TODO: 2 Add additional condition of travel for N meters before the HALT button is armed
            
            if (keypad.pressed == true) { // && started
                fputs(">>>>>>>>>>>>>>>>>>>>>>> HALT\n", stdout);
                display.status("HALT.");
                navDone = true;
                goGoGo = false;
                keypad.pressed = false;
                endRun();
            }
        } else {
            if (keypad.pressed == true) {
                fputs(">>>>>>>>>>>>>>>>>>>>>>> GO GO GO\n", stdout);
                display.status("GO GO GO!");
                goGoGo = true;
                keypad.pressed = false;
                beginRun();
            }
        }        

        // Are we at the last waypoint?
        // 
        if (fifo_first()->nextWaypoint == config.wptCount) {
            fputs("Arrived at final destination.\n", stdout);
            display.status("Arrived at end.");
            navDone = true;
            endRun();
        }

        //////////////////////////////////////////////////////////////////////////////
        // TELEMETRY
        //////////////////////////////////////////////////////////////////////////////
        if (timer.read_ms() > nextTelemUpdate) {
			SystemState *s = fifo_first();
			telem.sendPacket(s); // TODO 4 run this out of timer interrupt
			nextTelemUpdate += 200; // TODO 3 increase update speed
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
    
    fputs("Entering compass calibration in 2 seconds.\nLaunch _3DScatter Processing app now... type e to exit\n", stdout);
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
            	static char buf[2] = { 0xde, 0 }; // Start of transmission character
            	fputs(buf, stdout);
            	fputs(cvitos(m[0]), stdout);
            	fputs(" ", stdout);
            	fputs(cvitos(m[1]), stdout);
            	fputs(" ", stdout);
            	fputs(cvitos(m[2]), stdout);
            	fputs(" \r\n", stdout);

            	fputs(cvitos(m[0]), fp);
            	fputs(", ", stdout);
            	fputs(cvitos(m[1]), fp);
            	fputs(", ", stdout);
            	fputs(cvitos(m[2]), fp);
            	fputs("\n", fp);
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

    fputs("Entering gyro swing...\n", stdout);
    display.status("Starting...");
    wait(2);
    fp = openlog("gy");
    wait(2);
    display.status("Begin. Select exits.");

    fputs("Begin clockwise rotation, varying rpm... press select to exit\n", stdout);

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
        if (fp) {
        	fputs(cvitos(timer.read_ms()), fp);
        	fputs(",", fp);
        	fputs(cvitos(2*sensors.rightTotal), fp);
        	fputs(",", fp);
        	fputs(cvitos(sensors.g[0]), fp);
        	fputs(",", fp);
        	fputs(cvitos(sensors.g[1]), fp);
        	fputs(",", fp);
        	fputs(cvitos(sensors.g[2]), fp);
        	fputs(",", fp);
        	fputs(cvitos(sensors.gTemp), fp);
        	fputs("\n", fp);
        }
        wait(0.200);
    }    
    if (fp) {
        fclose(fp);
        display.status("Done. Saved.");
        fputs("Data collection complete.\n", stdout);
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

    fputs("Entering compass swing...\n", stdout);
    display.status("Starting...");
    wait(2);
    fp = openlog("sw");
    wait(2);
    display.status("Ok. Begin.");

    fputs("Begin clockwise rotation... exit after ", stdout);
    fputs(cvntos(revolutions), stdout);
	fputs(" revolutions\n", stdout);

    timer.reset();
    timer.start();

    // wait for index to change
    while ((leftCount += sensors._left.read()) < 2) {
        if (keypad.pressed) {
            keypad.pressed = false;
            break;    
        }
    }
    fputs(">>>> Index detected. Starting data collection\n", stdout);
    leftCount = 0;
    // TODO 3 how to parameterize status?
    lcd.pos(0,1);
    // TODO 3 lcd.printf("%1d %-14s", revolutions, "revs left");

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
            fputs(">>>>> ", stdout);
            fputs(cvitos(revolutions), stdout);
            fputs(" left\n", stdout); // we sense the rising and falling of the index so /2
            lcd.pos(0,1);
            // TODO 4 lcd.printf("%1d %-14s", revolutions, "revs left");
        }
        
        float heading2d = 180 * atan2((float) sensors.mag[1], (float) sensors.mag[0]) / PI;
        // Print out data
        fputs(cvitos(heading), stdout);
        fputs(cvftos(heading2d, 4), stdout);

        if (fp) {
        	fputs(cvitos(timer.read_ms()), stdout);
        	fputs(cvitos(heading), stdout);
        	fputs(cvftos(heading2d, 2), stdout);
        	fputs(cvftos(sensors.mag[0], 4), stdout);
        	fputs(cvftos(sensors.mag[1], 4), stdout);
        	fputs(cvftos(sensors.mag[2], 4), stdout);
        	fputs("\n", stdout);
        }

    }    
    if (fp) {
        fclose(fp);
        display.status("Done. Saved.");
        fputs("Data collection complete.\n", stdout);
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
#if 0
    char x;
    int count = 0;
    bool done=false;

    fputs("\nEntering serial bridge in 2 seconds, +++ to escape\n\n", stdout);
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
            fputc(serial.getc(), stdout);
        }
    }
#endif
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
#if 0
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

    fputs("press e to exit\n", stdout);
    while (!done) {
        int millis = timer.read_ms();

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
#endif
}


void telemetryMode() {
	RawSerial pc(USBTX, USBRX);
	bool done=false;
	int nextDisplayUpdate = 0;

	pc.baud(115200);

	pc.puts("Entering telemetry mode; press e to exit\n\n");

    timer.reset();
    timer.start();
    nextDisplayUpdate = timer.read_ms();

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

//        pc.printf("fifo in:%d out:%d\n", fifo_getInState(), fifo_getOutState());

        if (timer.read_ms() > nextDisplayUpdate) {
			SystemState *s = fifo_first();
			telem.sendPacket(s);
			nextDisplayUpdate += 100;
        }

    }
    endRun();

	return;
}


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
            // TODO 3 lcd.printf("%3d%%%-16s", backlight, "");
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
