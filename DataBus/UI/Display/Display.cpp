#include "mbed.h"
#include "devices.h"
#include "util.h"
#include "Display.h"

// TODO 3: move to Sparkfun class. This is for the Sparkfun module

#define WIDTH 16

#define LCD_COMMAND				0x7c
#define LCD_EXTENDED			0xfe
#define LCD_BACKLIGHT           0x80
#define LCD_CLEAR		        0x01
#define LCD_POS                 0x80
#define LCD_HOME       		    0x02

Display::Display(void):
	_lcd(LCDTX, LCDRX)
{
	// nothing to do yet
}

void Display::init()
{
	_lcd.baud(9600);
	// Set brightness
	for (int i=0; i < 2000; i++) {
		_lcd.putc(0x12);
	}
	wait(0.5);
	// 9600 baud
	_lcd.putc(LCD_COMMAND);
	_lcd.putc(0x0D);
	wait(0.5);
	// Set backlight
//	_lcd.putc(LCD_COMMAND);
//	_lcd.putc(150);
	// Clear screen
	_lcd.putc(LCD_EXTENDED);
	_lcd.putc(LCD_CLEAR);
	wait(0.5);
}

void Display::status(const char *st)
{
	int pad;
    char *s = (char *) st;
// TODO 3: parameterize screen width
	for (pad=WIDTH; pad > 0; pad--) {
		if (*s++ == '\0') break;
	}
	_lcd.putc(0xfe);
	_lcd.putc((1<<7)|64); // line 2
	_lcd.puts(st);
    while (pad--) {
    	_lcd.putc(' ');
    }
}

void Display::menu(const char *itemName)
{
	int pad;
	char *s = (char *) itemName;
	for (pad=WIDTH; pad > 0; pad--) {
		if (*s++ == '\0') break;
	}
	_lcd.putc(0xfe);
	_lcd.putc((1<<7)); // line 0, col 3
	_lcd.puts("< ");
	_lcd.puts(itemName);
	_lcd.puts(" >");
    pad -= 4; // account for "< " and " >"
    while (pad--) {
    	_lcd.putc(' ');
    }
}

void Display::select(const char *itemName)
{
	_lcd.putc(0xfe);
	_lcd.putc((1<<7));
	_lcd.puts(">>");
	_lcd.puts(itemName);
}

// display gauge at a given position (slot) along the bottom
void Display::gauge(int slot)
{
}

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


            
