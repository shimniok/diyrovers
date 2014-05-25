#include "mbed.h"
#include "util.h"
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
    lcd.puts("test\n"); // hopefully force 115200 on powerup
    lcd.clear();
    wait(0.3);
}

void Display::status(const char *st)
{
	int pad;
	char *s = (char *) st;
	for (pad=20; pad > 0; pad--) {
		if (*s++ == '\0') break;
	}
    lcd.pos(0,1);
    lcd.puts(st);
    while (pad--) {
    	lcd.puts(" ");
    }
}

void Display::menu(const char *itemName)
{
	int pad;
	char *s = (char *) itemName;
	for (pad=20; pad > 0; pad--) {
		if (*s++ == '\0') break;
	}
    lcd.pos(0,0);
    lcd.puts("< ");
    lcd.puts(itemName);
    lcd.puts(" >");
    pad -= 4; // account for "< " and " >"
    while (pad--) {
    	lcd.puts(" ");
    }
}

void Display::select(const char *itemName)
{
    lcd.pos(0,0);
    lcd.puts(">>");
	lcd.puts(itemName);
}

// display gauge at a given position (slot) along the bottom
void Display::gauge(int slot)
{
}

#define WIDTH 22

void Display::update(SystemState *state) {
	if (state) {
		lcd.pos(0,3);
		lcd.puts("V:");
		lcd.puts(cvftos(state->voltage, 1));
		lcd.puts(" G:");
		lcd.puts(cvftos(state->gpsHDOP, 1));
		lcd.puts(" ");
		lcd.puts(cvitos(state->gpsSats));

		lcd.pos(0,4);
		lcd.puts("H:");
		lcd.puts(cvftos(state->estHeading, 1));
		lcd.puts(" B:");
		lcd.puts(cvftos(state->bearing, 1));
		lcd.puts(" ");
		lcd.puts(cvftos(state->LABrg, 1));
	}
}

 void Display::redraw() { // TODO 3 rename
}


            
