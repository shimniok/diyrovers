/** Code for "Data Bus" UGV entry for Sparkfun AVC 2013
 *  http://www.bot-thoughts.com/
 */

///////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////////////////////////////

#include "mbed.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "globals.h"
#include "util.h"
#include "print.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Filesystem.h"
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
#include "Venus638flpx.h"
#include "Ublox6.h"
#include "Camera.h"
#include "PinDetect.h"
#include "Actuators.h"
#include "IncrementalEncoder.h"
#include "Steering.h"
#include "GeoPosition.h"
#include "Mapping.h"
#include "SimpleFilter.h"
#include "Beep.h"
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

// Display modes
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
DigitalOut ahrsStatus(LED4);            // AHRS status LED
//DigitalOut sonarStart(p18);             // Sends signal to start sonar array pings
Display display;                        // UI display
//Beep speaker(p24);                      // Piezo speaker

// INPUT
Menu menu;
Buttons keypad;

// VEHICLE
Steering steerCalc; 	  				// steering calculator

// COMM
Serial pc(USBTX, USBRX);                // PC usb communications
SerialGraphicLCD lcd(p17, p18, SD_FW);  // Graphic LCD with summoningdark firmware
//Serial *debug = &pc;

// SENSORS
Sensors sensors;                        // Abstraction of sensor drivers
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
//Schedule blink;

// Estimation & Navigation Variables
GeoPosition dr_here;                    // Estimated position based on estimated heading
Mapping mapper;

///////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////

int autonomousMode(void);
//void mavlinkMode(void);
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

extern "C" size_t xPortGetFreeHeapSize(void);

// TODO 2 move vApplicationStackOverflowHook to somewhere more appropriate
extern "C" void vApplicationStackOverflowHook( xTaskHandle xTask, signed char *pcTaskName ) {
	error("%% stack overflow in %s %%\n", pcTaskName);
}

// TODO 2 move to a more appropriate location, perhaps an lcd status task?
extern "C" void updateDisplay(void *args) {
	SystemState *s;
	display.redecorate();
	while (1) {
		// Pulling out current state so we get the most current
		s = fifo_first();
		// Now populate in the current GPS data
		//s->gpsHDOP = sensors.gps.hdop();
		//s->gpsSats = sensors.gps.sat_count();
		display.update(s);
		confStatus = !confStatus;
		vTaskDelay(1500);
	}
	return;
}

/*
extern "C" void userPanel(void *args) {
	bool printLCDMenu=true;

	while (1) {
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
                    // TODO 1 find a way to notify the shell and run the program
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
            display.redecorate();
            printLCDMenu = false;
        }

        vTaskDelay(50); // TODO: convert this to block waiting for keypad pressed
	}
	return;
}
*/

int main()
{
	// Send data back to the PC
    pc.baud(115200);
    fputs("Data Bus 2014\n", stdout);
    fflush(stdin);

    // Let's try setting priorities...
    //NVIC_SetPriority(DMA_IRQn, 0);
    NVIC_SetPriority(EINT0_IRQn, 5);    // wheel encoders
    NVIC_SetPriority(EINT1_IRQn, 5);    // wheel encoders
    NVIC_SetPriority(EINT2_IRQn, 5);    // wheel encoders
    NVIC_SetPriority(EINT3_IRQn, 5);    // wheel encoders
    NVIC_SetPriority(SPI_IRQn, 7);    	// uSD card, logging
    NVIC_SetPriority(TIMER3_IRQn, 8);   // updater running off Ticker
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

    fputs("Initializing...\n", stdout);
    display.status("Initializing");
    wait(0.2);
    
    // Initialize status LEDs
    ahrsStatus = 0;
    gpsStatus = 0;
    logStatus = 0;
    confStatus = 0;

    if (!fifo_init()) {
    	error("\n\n%% Error initializing SystemState fifo %%\n");
    }

    fputs("Loading configuration...\n", stdout);
    display.status("Load config");
    wait(0.2);
    if (config.load())                          // Load various configurable parameters, e.g., waypoints, declination, etc.
        confStatus = 1;

    // Calibrate throttle
	setThrottleMin(config.escMin);
	setThrottleMiddle(config.escZero);
	setThrottleMax(config.escMax);
	setThrottleScale(config.escScale);

	// Calibrate steering
    setSteerMiddle(config.steerZero);
    setSteerScale(config.steerScale);

    // Calibrate encoder
    sensors.Encoder_Calibrate(config.tireCircum, config.stripeCount);

    // Calibrate steering calculation
    steerCalc.setIntercept(config.interceptDist);
    steerCalc.setTrackWidth(config.trackWidth);
    steerCalc.setWheelbase(config.wheelbase);

    // Calibrate sensors
    sensors.Compass_Calibrate(config.magOffset, config.magScale);

    // Convert lat/lon waypoints to cartesian
    mapper.init(config.wptCount, config.wpt);
    for (unsigned int w = 0; w < MAXWPT && w < config.wptCount; w++) {
        mapper.geoToCart(config.wpt[w], &(config.cwpt[w]));
    }
    config.print();

    display.status("Nav configuration   ");
    steerCalc.setIntercept(config.interceptDist);               // Setup steering calculator based on intercept distance

    fputs("Calculating offsets...\n", stdout);
    display.status("Offset calculation  ");
    wait(0.2);
    // TODO 3 Really need to give the gyro more time to settle
    sensors.gps.disable();
    sensors.Calculate_Offsets();

    fputs("Starting GPS...\n", stdout);
    display.status("Start GPS           "); // TODO 3: would be nice not to have to pad at this level
    wait(0.2);
    sensors.gps.setUpdateRate(10);
    sensors.gps.enable();

    fputs("Starting Updater...\n", stdout);
    display.status("Start updater       ");
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

    fputs("Starting keypad...\n", stdout);

    keypad.init();
    
    fputs("Adding menu items...\n", stdout);

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

    display.status("Starting RTOS       ");
    wait(0.2);
	//checkit(__FILE__, __LINE__);
	xTaskCreate( shell, (const signed char * ) "shell", 700, NULL, (tskIDLE_PRIORITY+1), NULL );
	//xTaskCreate( userPanel, (const signed char * ) "panel", 150, NULL, (tskIDLE_PRIORITY+2), NULL );
	xTaskCreate( updateDisplay, (const signed char * ) "display", 100, NULL, (tskIDLE_PRIORITY+1), NULL );
    //checkit(__FILE__, __LINE__);
	vTaskStartScheduler(); // should never get past this line.
	error("%% scheduler start failure %%\n");

#if 1==0
    char cmd;
    bool printMenu = true;
    bool printLCDMenu = true;

    timer.start();
    timer.reset();

    int thisUpdate = timer.read_ms();
    int nextUpdate = thisUpdate;
    //int hdgUpdate = nextUpdate;

    while (1) {

        /*
        if (timer.read_ms() > hdgUpdate) {
            fprintf(stdout, "He=%.3f %.5f\n", kfGetX(0), kfGetX(1));
            hdgUpdate = timer.read_ms() + 100;
        }*/

    	/*
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
        */
        
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
            display.redecorate();
            printLCDMenu = false;
        }
        
        if (printMenu) {
            int i=0;
            fprintf(stdout, "\n==============\nData Bus Menu\n==============\n");
            fprintf(stdout, "%d) Autonomous mode\n", i++);
            fprintf(stdout, "%d) Bridge serial to GPS\n", i++);
            fprintf(stdout, "%d) Calibrate compass\n", i++);
            fprintf(stdout, "%d) Swing compass\n", i++);
            fprintf(stdout, "%d) Gyro calibrate\n", i++);
            fprintf(stdout, "%d) Instrument check\n", i++);
            fprintf(stdout, "%d) Display AHRS\n", i++);
            fprintf(stdout, "%d) Mavlink mode\n", i++);
            fprintf(stdout, "%d) Shell\n", i++);
            fprintf(stdout, "R) Reset\n");
            fprintf(stdout, "\nSelect from the above: ");
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
            fprintf(stdout, "%c\n", cmd);
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
                    display.select("Shell");
                    display.status("Standby.");
                    shell(0);
                    break;
                case '9' :
                    display.select("Serial bridge 2");
                    display.status("Standby.");
                    //gps2.enableVerbose();
                    //serialBridge( *(gps2.getSerial()) );
                    //gps2.disableVerbose();
                    break;
                default :
                    break;
            } // switch        


        } // if (pc.readable())

        wait(0.1);

    } // while
#endif

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

    ahrsStatus = 0;
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
    if (fp == NULL) return -1;

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

    fputs("Entering gyro swing...\n", stdout);
    display.status("Starting...");
    wait(2);
    fp = openlog("gy");
    if (fp == NULL) return -1;

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
        	printInt(fp, timer.read_ms());

        	printInt(fp, 2*sensors.rightTotal);
        	fputc(',', stdout);
        	printInt(fp, sensors.g[0]);
        	fputc(',', stdout);
        	printInt(fp, sensors.g[1]);
        	fputc(',', stdout);
        	printInt(fp, sensors.g[2]);
        	fputc(',', stdout);
        	printInt(fp, sensors.gTemp);
        	fputc('\n', stdout);
        }
        wait(0.200); // TODO 2 vTaskDelay()
    }    
    if (fp) {
        fclose(fp);
        display.status("Done. Saved.");
        fputs("Data collection complete.\n", stdout);
        wait(2); // TODO 2 vTaskDelay()
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
    if (fp == NULL) return -1;

    wait(2);
    display.status("Ok. Begin.");

    fputs("Begin clockwise rotation... exit after ", stdout);
    printInt(stdout, revolutions);
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
    lcd.printInt(revolutions);
    lcd.puts(" revs left    ");

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
            printInt(stdout, revolutions);
            fputs(" left\n", stdout); // we sense the rising and falling of the index so /2
            lcd.pos(0,1);
            lcd.printInt(revolutions);
            lcd.puts("revs left     ");
        }
        
        float heading2d = 180 * atan2((float) sensors.mag[1], (float) sensors.mag[0]) / PI;
        // Print out data
        //getRawMag(m);
        printInt(stdout, heading);
        printFloat(stdout, heading2d, 4);
        fputc('\n', stdout);

//        int t1=t.read_us();
        if (fp) {
        	printInt(fp, timer.read_ms());
        	printInt(fp, heading);
        	printFloat(fp, heading2d, 2);
        	printFloat(fp, sensors.mag[0], 4);
        	printFloat(fp, sensors.mag[1], 4);
        	printFloat(fp, sensors.mag[2], 4);
        }
//        int t2=t.read_us();
//        fprintf(stdout, "dt=%d\n", t2-t1);
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

// TODO 3 fix serial bridge, probably won't work like this, I don't know.
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
        
        // TODO 4 would be nice if we could use streams, fcntl non-blocking, etc.
        while (pc.readable()) {
            if (pc.getc() == 'e') {
                done = true;
                break;
            }
        }

        // TODO 3 use vTaskDelay here
		if ((millis % 1000) == 0) {
			SystemState *s = fifo_first();

			// update time
			fputs("\nupdate() time = ", stdout);
			printFloat(stdout, getUpdateTime()/1000.0, 3);
			fputs(" msec\n", stdout);

			// rangers
			fputs("Rangers: L=", stdout);
			printFloat(stdout, sensors.leftRanger, 2);
			fputs(" R=", stdout);
			printFloat(stdout, sensors.rightRanger, 2);
			fputs(" C=", stdout);
			printFloat(stdout, sensors.centerRanger, 2);
			fputc('\n', stdout);

			// gyro, raw
			fputs("g=(", stdout);
			printInt(stdout, sensors.g[0]);
			fputc(',', stdout);
			printInt(stdout, sensors.g[1]);
			fputc(',', stdout);
			printInt(stdout, sensors.g[2]);
			fputs(") ", stdout);
			printInt(stdout, sensors.gTemp);
			fputc('\n', stdout);

			// gyro, corrected
			fputs("gc=(", stdout);
			printInt(stdout, sensors.gyro[0]);
			fputc(',', stdout);
			printInt(stdout, sensors.gyro[1]);
			fputc(',', stdout);
			printInt(stdout, sensors.gyro[2]);
			fputs(")\n", stdout);

			// accelerometer
			fputs("a=(", stdout);
			printInt(stdout, sensors.a[0]);
			fputc(',', stdout);
			printInt(stdout, sensors.a[1]);
			fputc(',', stdout);
			printInt(stdout, sensors.a[2]);
			fputs(")\n", stdout);

			// heading
			fputs("estHdg=", stdout);
			printFloat(stdout, s->estHeading, 2);
			fputs(" lagHdg=", stdout);
			printFloat(stdout, s->estLagHeading, 2);
			fputc('\n', stdout);

			// speed
			fputs("speed: left=", stdout);
			printFloat(stdout, sensors.lrEncSpeed, 2);
			fputs("  right=", stdout);
			printFloat(stdout, sensors.rrEncSpeed, 2);
			fputc('\n', stdout);

			// gps
			fputs("gps=(", stdout);
			printFloat(stdout, sensors.gps.latitude(), 6);
			fputs(", ", stdout);
			printFloat(stdout, sensors.gps.longitude(), 6);
			fputs(", ", stdout);
			printFloat(stdout, sensors.gps.heading_deg(), 1);
			fputs(", ", stdout);
			printFloat(stdout, sensors.gps.speed_mps(), 1);
			fputs(", ", stdout);
			printFloat(stdout, sensors.gps.hdop(), 1);
			fputs(", ", stdout);
			printFloat(stdout, sensors.gps.sat_count(), 1);
			fputs(", ", stdout);
			printInt(stdout, sensors.gps.getAvailable());
			fputc('\n', stdout);

			// nav
			fputs("brg=", stdout);
			printFloat(stdout, s->bearing, 2);
			fputs(" d=", stdout);
			printFloat(stdout, s->distance, 4);
			fputs(" sa=", stdout);
			printFloat(stdout, s->steerAngle, 2);
			fputc('\n', stdout);

			// power
			fputs("v=", stdout);
			printFloat(stdout, sensors.voltage, 2);
			fputs(" a=", stdout);
			printFloat(stdout, sensors.current, 2);
			fputc('\n', stdout);
		}

#if 0==1
		/* TODO 3 figure out how/where to do instrument check. */
		if ((millis % 3000) == 0) {

			lcd.pos(0,2);
			lcd.puts("G=");
			lcd.printFloat(sensors.gyro[0], 1);
			lcd.puts(",");
			lcd.printFloat(sensors.gyro[1], 1);
			lcd.puts(",");
			lcd.printFloat(sensors.gyro[2], 1);
			lcd.puts("      ");
			vTaskDelay(10);

			lcd.pos(0,3);
			lcd.puts("La=");
			lcd.printFloat(sensors.gps.latitude(), 7);
			lcd.puts(" HD=");
			lcd.printFloat(sensors.gps.hdop(), 1);
			lcd.puts("      ");
			vTaskDelay(10);

			lcd.pos(0,4);
			lcd.puts("Lo=");
			lcd.printFloat(sensors.gps.longitude(), 7);
			lcd.puts(" Sat=");
			lcd.printFloat(sensors.gps.sat_count(), 1);
			lcd.puts("      ");
			vTaskDelay(10);

			lcd.pos(0,5);
			lcd.puts("V=");
			lcd.printFloat(sensors.voltage, 2);
			lcd.puts(" A=");
			lcd.printFloat(sensors.current, 3);
			lcd.puts("      ");
			vTaskDelay(10);

		}
#endif

    } // while !done
    // clear input buffer
    fflush(stdin);
    lcd.clear();
    ahrsStatus = 0;
    gpsStatus = 0;
}


#if 0==1
// TODO: 3 move Mavlink into task
// possibly also buffered if necessary
void mavlinkMode() {
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

    fprintf(stdout, "Entering MAVlink mode; reset the MCU to exit\n\n");

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
    
    fputc('\n', stdout);
    
    return;
}
#endif

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
            lcd.printInt(backlight);
            lcd.putc('%');
            lcd.puts("   ");
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
