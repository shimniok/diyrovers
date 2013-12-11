#include "mbed.h"
#include "Display.h"

// TODO 3 would also be nice if we did all the printf crap here too

#define LCD_FMT "%-20s" // used to fill a single line on the LCD screen

// This is for the Sparkfun module
#define DISPLAY_CLEAR     0x01
#define DISPLAY_SET_POS   0x08

Display::Display(void): 
        lcd(p17, p18, SD_FW),
        v(1, 20, 35, 20, 'V'),
        //a(11, 40, 15, 'A'),
        g1(22, 20, 35, 20, 'G'),
        g2(43, 20, 35, 20, 'H')
{
  // nothin to do
}

void Display::init()
{
    lcd.baud(115200);
    lcd.printf("test\n"); // hopefully force 115200 on powerup
    lcd.clear();
    wait(0.3);
    
    // Initialize LCD graphics
    Bargraph::lcd = &lcd;   
    v.calibrate(6.3, 8.4);
    a.calibrate(0, 15.0);
    g1.calibrate(0, 10.0);
    g2.calibrate(4.0, 0.8);
    //GPSStatus g2(21, 12);
    //GPSStatus::lcd = &lcd;

}

void Display::status(const char *st)
{
    lcd.pos(0,1);
    lcd.printf(LCD_FMT, st);
}

void Display::menu(const char *itemName)
{
    lcd.pos(0,0);
    lcd.printf("< %-14s >", itemName);
}

void Display::select(const char *itemName)
{
    lcd.pos(0,0);
    lcd.printf(">>%-14s", itemName);
}

// display gauge at a given position (slot) along the bottom
void Display::gauge(int slot)
{
}


void Display::update(SystemState state) {
    v.update(state.voltage);
    //a.update(state.current);
    g1.update((float) state.gpsSats);
    g2.update(state.gpsHDOP);

    lcd.rectFill(68,16,122,64, 0x00);
    lcd.circle(90, 40, 22, true);
    lcd.circle(90, 40, 14, true);
    lcd.posXY(90-9,40-(8/2)); // 3 * 6 / 2, char width=6, 5 chars, half that size
    lcd.printf("%03.0f", state.estHeading);
    int nx = 90 - 18 * sin(3.141529 * state.estHeading / 180.0);
    int ny = 40 - 18 * cos(-3.141529 * state.estHeading / 180.0);
    lcd.posXY(nx - 2, ny - 3);
    lcd.printf("N");
    //lcd.circle(nx, ny, 5, true);
    int bx = 90 - 18 * sin(-3.141529 * (state.bearing-state.estHeading) / 180.0);
    int by = 40 - 18 * cos(-3.141529 * (state.bearing-state.estHeading) / 180.0);
    lcd.circle(bx, by, 2, true);
    
    /*
    lcd.posXY(60, 22);
    lcd.printf("%.2f", state.rightRanger);
    lcd.posXY(60, 32);
    lcd.printf("%.2f", state.leftRanger);
    lcd.posXY(60, 42);
    lcd.printf("%5.1f", state.estHeading);
    lcd.posXY(60, 52);
    lcd.printf("%.3f", state.gpsCourse);
    */
    // TODO: 3 address integer overflow
    // TODO: 3 display scheduler() timing
}

 void Display::redraw() { // TODO 3 rename
    v.init();
    //a.init();
    g1.init();
    g2.init();
    /*
    lcd.posXY(50, 22);
    lcd.printf("R");
    lcd.rect(58, 20, 98, 30, true);
    wait(0.01);
    lcd.posXY(50, 32);
    lcd.printf("L");
    lcd.rect(58, 30, 98, 40, true);
    wait(0.01);
    lcd.posXY(50, 42);
    lcd.printf("H");
    lcd.rect(58, 40, 98, 50, true);
    wait(0.01);
    lcd.posXY(44,52);
    lcd.printf("GH");
    lcd.rect(58, 50, 98, 60, true);
    */
}


            
