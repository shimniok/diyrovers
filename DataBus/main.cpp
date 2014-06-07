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

int autonomousMode(void);
void serialBridge(Serial &gps);
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
    pc.puts("Gyro scale: ");
    pc.puts(cvftos(config.gyroScale, 5));
    pc.puts("\n");
    sensors.setGyroScale(config.gyroScale);

    display.status("GPS configuration   ");
    pc.puts("GPS valid speed: ");
    pc.puts(cvftos(config.gpsValidSpeed,1));
    pc.puts("\n");

    pc.puts("Gyro config         ");
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
    menu.add("Gyro Calib", &gyroSwing);
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
            fputs("2) Gyro calibrate\n", stdout);
            fputs("3) Shell\n", stdout);
            fputs("R) Reset\n", stdout);
            fputs("\nSelect from the above: ", stdout);
            fflush(stdout);
            printMenu = false;
        }

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
                    display.select("Gyro Calib");
                    display.select(menu.getItemName(2));
                    gyroSwing();
                    break;
                case '3' :
                    display.select("Shell");
                    display.status("Standby.");
                    shell(0);
                    break;
                default :
                    break;
            } // switch        

		} // if (pc.readable())

        wait(0.1);

    } // while

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

// Gather gyro data using turntable equipped with dual channel
// encoder. Use onboard wheel encoder system. Left channel
// is the index (0 degree) mark, while the right channel
// is the incremental encoder.  Can then compare gyro integrated
// heading with machine-reported heading
//
int gyroSwing()
{
    FILE *fp;
    int now;
    int next;
    int g[3];
    int leftTotal=0;
    int rightTotal=0;

    // Timing is pretty critical so just in case, disable serial processing from GPS
    sensors.gps.disable();
    stopUpdater();

    fputs("Starting gyro swing...\n", stdout);
    display.status("Starting...");
//    fp = openlog("gy");
    fp = stdout;

    display.status("Rotate clockwise.");
    fputs("Begin clockwise rotation, varying rpm\n", stdout);
    wait(1);

    display.status("Select exits.");
	fputs("Press select to exit\n", stdout);
    wait(1);


    timer.reset();
    timer.start();

    next = now = timer.read_ms();

    sensors._right.read();  // easiest way to reset the heading counter
    sensors._left.read();
    
    while (1) {
    	now = timer.read_ms();

    	if (keypad.pressed) {
            keypad.pressed = false;
            break;
        }

        if (now >= next) {
            leftTotal += sensors._left.read();
            rightTotal += sensors._right.read();
        	sensors._gyro.read(g);
			fputs(cvitos(now), fp);
			fputs(" ", fp);
			fputs(cvntos(leftTotal), fp);
			fputs(" ", fp);
			fputs(cvntos(rightTotal), fp);
			fputs(" ", fp);
			fputs(cvitos(g[_z_]), fp);
			fputs("\n", fp);
			next = now + 50;
        }
    }    
    if (fp && fp != stdout) {
        fclose(fp);
        display.status("Done. Saved.");
        fputs("Data collection complete.\n", stdout);
        wait(2);
    }

    sensors.gps.enable();
    restartNav();
    startUpdater();
    
    keypad.pressed = false;

    return 0;
}



void bridgeRecv()
{
#if 0
    while (dev && dev->readable()) {
        pc.putc(dev->getc());
    }
#endif
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
