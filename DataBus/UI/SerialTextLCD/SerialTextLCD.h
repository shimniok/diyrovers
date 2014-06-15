/*
 * SerialTextLCD.h
 *
 *  Created on: Jun 14, 2014
 *      Author: mes
 */

#ifndef SERIALTEXTLCD_H_
#define SERIALTEXTLCD_H_

#include "mbed.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

/** Serial Text LCD/OLED Library
 * www.digole.com/index.php?productID=535
 *
 * Includes Arduino Print class member functions
 */
class SerialTextLCD {
public:

    /** Create a new Serial LCD interface
     *
     * @param tx is the pin for UART tx
     * @param rx is the pin for UART rx
     */
    SerialTextLCD(PinName rx, PinName tx);


    /** Write out a raw character
     * @param x is the character to write
     * @returns 1
     */
    size_t write(const char x);


    /** Write out raw data from a buffer
     * @param buffer is the char array to write
     * @param size is the the number of bytes to write
     * @returns size
     */
    size_t write(const char *buffer, size_t size);


    /** Write out raw string
     * @param str is the string to write
     * @returns number of bytes written
     */
    size_t write(const char *str);


    /** Prints a char to the display in a single I2C transmission using "TTb\0"
     *
     * @param c is the character to print
     * @returns 1
     */
    size_t print(const char c);


    /** Prints a string of data to the display in a single I2C transmission using "TTbbb...\0"
     *
     * @param s is the null-terminated char array to print
     * @returns length of s
     */
    size_t print(const char s[]);


    /** Print out an unsigned char as a number
     * @param u is the integer to print
     * @param base is the base to print, either DEC (default), HEX, BIN
     * @returns number of chars written
     */
    size_t print(unsigned char u, int base = DEC);


    /** Print out an integer
     * @param i is the integer to print
     * @param base is the base to print, either DEC (default), HEX, BIN
     * @returns number of chars written
     */
    size_t print(int i, int base = DEC);


    /** Print out an unsigned integer
     * @param u is the integer to print
     * @param base is the base to print, either DEC (default), HEX, BIN
     * @returns number of chars written
     */
    size_t print(unsigned int u, int base = DEC);


    /** Print out a long as a number
     * @param l is the integer to print
     * @param base is the base to print, either DEC (default), HEX, BIN
     * @returns number of chars written
     */
    size_t print(long l, int base = DEC);


    /** Print out an unsigned long
     * @param l is the integer to print
     * @param base is the base to print, either DEC (default), HEX, BIN
     * @returns number of chars written
     */
    size_t print(unsigned long l, int base = DEC);


    /** Print out a double
     * @param f is the integer to print
     * @param digits is the number of digits after the decimal
     */
    size_t print(double f, int digits = 2);


    /** Prints a string of data to the display in a single I2C transmission using "TTbbb...\0"
     *
     * @param s is the null-terminated char array to print
     * @returns length of s
     */
    size_t println(const char s[]);


    /** Prints a char the display in a single I2C transmission using "TTb\0"
     *
     * @param c is the character to print
     * @returns 1
     */
    size_t println(char c);


    /** Prints an unsigned char as a number
     *
     * @param u is the unsigned char number
     * @returns 1
     */
    size_t println(unsigned char u, int base = DEC);


    /** Print out an integer
     * @param i is the integer to print
     * @param base is the base to print, either DEC (default), HEX, BIN
     * @returns number of chars written
     */
    size_t println(int i, int base = DEC);


    /** Print out an unsigned char as a number
     * @param u is the integer to print
     * @param base is the base to print, either DEC (default), HEX, BIN
     * @returns number of chars written
     */
    size_t println(unsigned int u, int base = DEC);


    /** Print out a long as a number
     * @param l is the integer to print
     * @param base is the base to print, either DEC (default), HEX, BIN
     * @returns number of chars written
     */
    size_t println(long l, int base = DEC);


    /** Print out an unsigned long
     * @param l is the integer to print
     * @param base is the base to print, either DEC (default), HEX, BIN
     * @returns number of chars written
     */
    size_t println(unsigned long l, int base = DEC);


    /** Print out a double
     * @param f is the integer to print
     * @param digits is the number of digits after the decimal
     * @returns number of chars written
     */
    size_t println(double f, int digits = 2);


    /** prints, well, nothing in this case, but pretend we printed a newline
     * @returns 1
     */
    size_t println(void);


    /*---------functions for Text and Graphic LCD adapters---------*/

    /** Turns off the cursor */
    void disableCursor(void);

    /** Turns on the cursor */
    void enableCursor(void);

    /** Displays a string at specified coordinates
     * @param x is the x coordinate to display the string
     * @param y is the y coordinate to display the string
     * @param s is the string to display
     */
    void drawStr(uint8_t x, uint8_t y, const char *s);

    /** Sets the print position for text
     * @param x is the x coordinate to display the string
     * @param y is the y coordinate to display the string
     */
    void pos(uint8_t x, uint8_t y);

    /** Clears the display screen */
    void clear(void);

    /** Configure your LCD if other than 1602 and the chip is other than KS0066U/F / HD44780
     * @param col is the number of columns
     * @param row is the number of rows
     */
    void setLCDColRow(uint8_t col, uint8_t row);

    /** Display Config on/off, the factory default set is on,
     * so, when the module is powered up, it will display
     * current communication mode on LCD, after you
     * design finished, you can turn it off
     * @param v is the 1 is on, 0 is off
     */
    void displayConfig(uint8_t v);

    /** Holdover from Arduino library; not needed */
    void preprint(void);

    /** go to next text line, depending on the font size */
    void nextTextLine(void);

    /** set color for graphic function */
    void setColor(uint8_t);

    /** Turn on back light */
    void backLightOn(void);

    /** Turn off back light */
    void backLightOff(void);

    /** send command to LCD drectly
     * @param d - command
     */
    void directCommand(uint8_t d);

    /** send data to LCD drectly
     * @param d is the data
     */
    void directData(uint8_t d);

    /** set text position back to previous, only one back allowed */
    void setTextPosBack(void);

    void setTextPosOffset(char xoffset, char yoffset);

    /** set absolute text position */
    void setTextPosAbs(uint8_t x, uint8_t y);

    /** Set backlight pwm */
    void backlight(uint8_t b);

private:
    char buf[128];

    size_t printNumber(unsigned long n, uint8_t base);
    size_t printFloat(double number, uint8_t digits);
};



#endif /* SERIALTEXTLCD_H_ */
