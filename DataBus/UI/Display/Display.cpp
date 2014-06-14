#include "mbed.h"
#include "util.h"
#include "Display.h"

// TODO 3: move to Sparkfun class. This is for the Sparkfun module
#define DISPLAY_CLEAR     0x01
#define DISPLAY_SET_POS   0x08

Display::Display(void)
{
	// nothin to do
}

void Display::init()
{
//    for (int i=0; i < 100; i++) {
//    	_lcd.print("test\n"); // hopefully force 115200 on powerup
//    }
//	_lcd.clear();
//    wait(0.3);
}

void Display::status(const char *st)
{
//	int pad;
//	char *s = (char *) st;
//	// TODO 3: parameterize screen width
//	for (pad=15; pad > 0; pad--) {
//		if (*s++ == '\0') break;
//	}
//    lcd.pos(0,1);
//    lcd.print(st);
//    while (pad--) {
//    	lcd.print(" ");
//    }
}

void Display::menu(const char *itemName)
{
//	int pad;
//	char *s = (char *) itemName;
//	for (pad=20; pad > 0; pad--) {
//		if (*s++ == '\0') break;
//	}
//    lcd.pos(0,0);
//    lcd.print("< ");
//    lcd.print(itemName);
//    lcd.print(" >");
//    pad -= 4; // account for "< " and " >"
//    while (pad--) {
//    	lcd.print(" ");
//    }
}

void Display::select(const char *itemName)
{
//    lcd.pos(0,0);
//    lcd.print(">>");
//	lcd.print(itemName);
}

// display gauge at a given position (slot) along the bottom
void Display::gauge(int slot)
{
}

#define WIDTH 22

void Display::update(SystemState *state) {
#if 0
	if (state) {
		// TODO 2 fix padding/overwrite
		_lcd.pos(0,2);
		_lcd.print("V:");
		_lcd.print(cvftos(state->voltage, 1));
		_lcd.print(" G:");
		_lcd.print(cvftos(state->gpsHDOP, 1));
		_lcd.print(" ");
		_lcd.print(cvitos(state->gpsSats));

		_lcd.pos(0,3);
		_lcd.print("H:");
		_lcd.print(cvftos(state->estHeading, 1));
		_lcd.print(" B:");
		_lcd.print(cvftos(state->bearing, 1));
		_lcd.print(" ");
		_lcd.print(cvftos(state->LABrg, 1));
	}
#endif
}

void Display::redraw() { // TODO 3 rename
}


            
