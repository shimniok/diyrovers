/** Digole Serial Display library, I2C
 *
 * @Author: Digole Digital Solutions : www.digole.com ported from Arduino to mbed by Michael Shimniok www.bot-thoughts.com
 */
#ifndef DigoleSerialDisp_h
#define DigoleSerialDisp_h

#include "mbed.h"
#include <inttypes.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define delay(x) wait_ms(x)

// Communication set up command
// Text function command
// Graph function command

#define Serial_UART 0;
#define Serial_I2C 1;
#define Serial_SPI 2;
#define _TEXT_ 0
#define _GRAPH_ 1

/** Digole Serial LCD/OLED Library
 * www.digole.com/index.php?productID=535
 *
 * Includes Arduino Print class member functions
 */
class DigoleSerialDisp {
public:

    /** Create a new Digole Serial Display interface
     *
     * @param sda is the pin for I2C SDA
     * @param scl is the pin for I2C SCL
     * @param address is the 7-bit address (default is 0x27 for the device)
     */
    DigoleSerialDisp(PinName sda, PinName scl, uint8_t address=0x27);


    /** Carryover from Arduino library, not needed
     */
    void begin(void) { } // nothing to do here


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
    
    /** Sets the print position for graphics or text
     * @param x is the x coordinate to display the string
     * @param y is the y coordinate to display the string
     * @param graph if set to _TEXT_ affects subsequent text position, otherwise, affects graphics position
     */
    void setPrintPos(uint8_t x, uint8_t y, uint8_t graph = _TEXT_);
    
    /** Clears the display screen */
    void clearScreen(void);
    
    /** Configure your LCD if other than 1602 and the chip is other than KS0066U/F / HD44780 
     * @param col is the number of columns
     * @param row is the number of rows
     */
    void setLCDColRow(uint8_t col, uint8_t row);
    
    /** Sets a new I2C address for the display (default is 0x27), the adapter will store the new address in memory
     * @param address is the the new address 
     */
    void setI2CAddress(uint8_t add);
    
    /** Display Config on/off, the factory default set is on, 
     * so, when the module is powered up, it will display 
     * current communication mode on LCD, after you 
     * design finished, you can turn it off
     * @param v is the 1 is on, 0 is off
     */
    void displayConfig(uint8_t v);
    
    /** Holdover from Arduino library; not needed */
    void preprint(void);
    
    /*----------Functions for Graphic LCD/OLED adapters only---------*/
    //the functions in this section compatible with u8glib
    void drawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *bitmap);
    void setRot90(void);
    void setRot180(void);
    void setRot270(void);
    void undoRotation(void);
    void setRotation(uint8_t);
    void setContrast(uint8_t);
    void drawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
    void drawCircle(uint8_t x, uint8_t y, uint8_t r, uint8_t = 0);
    void drawDisc(uint8_t x, uint8_t y, uint8_t r);
    void drawFrame(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
    void drawPixel(uint8_t x, uint8_t y, uint8_t = 1);
    void drawLine(uint8_t x, uint8_t y, uint8_t x1, uint8_t y1);
    void drawLineTo(uint8_t x, uint8_t y);
    void drawHLine(uint8_t x, uint8_t y, uint8_t w);
    void drawVLine(uint8_t x, uint8_t y, uint8_t h);
    //-------------------------------
    //special functions for our adapters
    
    /** Sets the font
     *
     * @parameter font - available fonts: 6,10,18,51,120,123, user font 200-203
     */
    void setFont(uint8_t font);
    
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
    
    /** Move rectangle area on screen to another place
     * @param x0, y1 is the top left of the area to move
     * @param x1, y1 is the bottom right of the area to move
     * @param xoffset, yoffset is the the distance to move
     */
    void moveArea(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, char xoffset, char yoffset);

    /** Display startup screen */
    void displayStartScreen(uint8_t m);
    
    /** Set display mode */
    void setMode(uint8_t m);

    /** set text position back to previous, only one back allowed */
    void setTextPosBack(void);    
    
    void setTextPosOffset(char xoffset, char yoffset);
    void setTextPosAbs(uint8_t x, uint8_t y);
    void setLinePattern(uint8_t pattern);
    /** Only for universal serial adapter */
    void setLCDChip(uint8_t chip);


    /** Set Start Screen, 1st B is the lower byte of data length. 
     * Convert images to C array here: <a href="http://www.digole.com/tools/PicturetoC_Hex_converter.php">http://www.digole.com/tools/PicturetoC_Hex_converter.php</a>
     * @param lon is the length of data
     * @param data is the binary data
     */
    void uploadStartScreen(int lon, const unsigned char *data); //upload start screen
    
    /** Upload a user font
     * @param lon is the length of data
     * @param data is the user font data
     * @param sect is the section of memory you want to upload to
     */
    void uploadUserFont(int lon, const unsigned char *data, uint8_t sect); //upload user font

    /** Send a Byte to output head on board
     * @param x is the byte to output
     */
    void digitalOutput(uint8_t x);

private:
    I2C _device;
    uint8_t _address;
    uint8_t _Comdelay;
    char buf[128];

    size_t printNumber(unsigned long n, uint8_t base);
    size_t printFloat(double number, uint8_t digits);
};

#endif
