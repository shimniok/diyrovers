/* Serial Graphics LCD Driver for Sparkfun Serial Graphics LCD, LCD-09351; and Graphic 
 * LCD Serial Backpack, LCD-09352.
 *
 * @author Michael Shimniok http://www.bot-thoughts.com/
 *
 */
#ifndef _SERIALGRAPHICLCD_H
#define _SERIALGRAPHICLCD_H

#include "mbed.h"

/** Firmware modes */
#define SFE_FW 0x01       // Stock SFE firmware
#define SD_FW  0x02       // summoningdark firmware http://sourceforge.net/projects/serialglcd/

/** Modes for SD firmware */
#define FILL 0xFF
#define CLEAR 0x00

/** LCD Baud Rates */
#define LCD_4800   1
#define LCD_9600   2
#define LCD_19200  3
#define LCD_38400  4
#define LCD_57600  5
#define LCD_115200 6

/** LCD Types */
#define LCD_128x64  1
#define LCD_160x128 2

/** Interface to the Sparkfun Serial Graphic LCD, LCD-09351; and Graphic 
 * LCD Serial Backpack, LCD-09352. Derived class from Serial so that you
 * can conveniently printf(), putc(), etc to the display.
 * 
 * Example:
 * @code
 * #include "mbed.h"
 * #include "SerialGraphicLCD.h"
 * 
 * SerialGraphicLCD lcd(p26, p25);
 * 
 * int main() {
 *   lcd.baud(115200); // default baud rate
 *   while (1) {
 *      lcd.clear();
 *      lcd.rect(3, 3, 20, 20);
 *      lcd.printf("Hello World!");
 *      lcd.pixel(14, 35, true);
 *      lcd.pixel(16, 36, true);
 *      lcd.pixel(18, 37, true);
 *      lcd.pos(5, 30);
 *      lcd.printf("Hi");
 *      lcd.circle(50, 20, 20, true);
 *      lcd.pos(50, 20);
 *      lcd.printf("Howdy");
 *      lcd.line(0, 0, 25, 25, true);
 *      wait(2);
 *   }
 * }
 * @endcode
 */
class SerialGraphicLCD: public Serial {
public:
    /** Create a new interface to a Serial Graphic LCD
     * Note that the display lower left corner is coordinates 0, 0.
     * Rows start at the top at 0, columns start at the left at 0.
     * @param tx -- mbed transmit pin
     * @param rx -- mbed receive pin
     */
    SerialGraphicLCD(PinName tx, PinName rx);
    
    /** Create a new interface to a Serial Graphic LCD
     * Note that the display lower left corner is coordinates 0, 0.
     * Rows start at the top at 0, columns start at the left at 0.
     * @param tx -- mbed transmit pin
     * @param rx -- mbed receive pin
     * @param firmware -- SFE_FW, stock firmware or SD_FW, summoningdark firmware
     */
    SerialGraphicLCD(PinName tx, PinName rx, int firmware);

    /** clear the screen
     */
    void clear(void);

    /** set text position in rows, columns
     *
     * @param col is the col coordinate
     * @param row is the row coordinate
     */
    void pos(int col, int row);
    
    /** set text position in x, y coordinates
     *
     * @param x is the x coordinate
     * @param y is the y coordinate
     */
    void posXY(int x, int y);
    
    /** set or erase a pixel
     *
     * @param x is the x coordinate
     * @param y is the y coordinate
     * @param set if true sets the pixel, if false, erases it
     */
    void pixel(int x, int y, bool set);
    
    /** draw or erase a line
     *
     * @param x1 is the x coordinate of the start of the line
     * @param y1 is the y coordinate of the start of the line
     * @param x2 is the x coordinate of the end of the line
     * @param y2 is the y coordinate of the end of the line
     * @param set if true sets the line, if false, erases it
     */
    void line(int x1, int y1, int x2, int y2, bool set);
    
    /** set or reset a circle
     *
     * @param x is the x coordinate of the circle center
     * @param y is the y coordinate of the circle center
     * @param r is the radius of the circle
     * @param set if true sets the pixel, if false, clears it
     */
    void circle(int x, int y, int r, bool set);
    
    /** draw a rectangle
     *
     * @param x1 is the x coordinate of the upper left of the rectangle
     * @param y1 is the y coordinate of the upper left of the rectangle
     * @param x2 is the x coordinate of the lower right of the rectangle
     * @param y2 is the y coordinate of the lower right of the rectangle
     */
    void rect(int x1, int y1, int x2, int y2);

    /** draw or erase a rectangle (SD firmware only)
     *
     * @param x1 is the x coordinate of the upper left of the rectangle
     * @param y1 is the y coordinate of the upper left of the rectangle
     * @param x2 is the x coordinate of the lower right of the rectangle
     * @param y2 is the y coordinate of the lower right of the rectangle
     */
    void rect(int x1, int y1, int x2, int y2, bool set);    

    /** Draw a filled box. 
     *
     * @param x1 is the x coordinate of the upper left of the rectangle
     * @param y1 is the y coordinate of the upper left of the rectangle
     * @param x2 is the x coordinate of the lower right of the rectangle
     * @param y2 is the y coordinate of the lower right of the rectangle
     * @param fillByte describes 1 8-pixel high stripe that is repeated every x 
     * pixels and every 8 y pixels. The most useful are CLEAR (0x00) to clear the box, and FILL (0xFF) to fill it.
     */
    void rectFill(int x1, int y1, int x2, int y2, char fillByte);
    
    /** erase a rectangular area
     *
     * @param x1 is the x coordinate of the upper left of the area
     * @param y1 is the y coordinate of the upper left of the area
     * @param x2 is the x coordinate of the lower right of the area
     * @param y2 is the y coordinate of the lower right of the area
     */
    void erase(int x1, int y1, int x2, int y2);
    
    /** set backlight duty cycle
     *
     * @param i is the duty cycle from 0 to 100; 0 is off, 100 is full power
     */
    void backlight(int i);
    
    /** clear screen and put in reverse mode
     */
    void reverseMode(void);

    /** configure the lcd baud rate so you have to call this along with baud() to change
     * communication speeds
     *
     * @param b is the baud rate, LCD_4800, LCD_9600, LCD_19200, LCD_38400, LCD_57600 or LCD_115200
     */
    void lcdbaud(int b);
    
    
    /** sets the resolution of the LCD so that the pos() call works properly
     * defaults to LCD_128x64.
     *
     * @param type is the type of LCD, either LCD_128x64 or LCD_160x128
     */
    void resolution(int type);

    /** sets the resolution of the LCD in x and y coordinates which determines
     * how rows and columns are calculated in the pos() call.  Defaults to
     * x=128, y=64
     *
     * @param x is the number of horizontal pixels
     * @param y is the number of vertical pixels
     */
    void resolution(int x, int y);
    
    private:
        int _xMax;
        int _yMax;
        int _firmware;
};


#endif