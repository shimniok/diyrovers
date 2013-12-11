/** Digole Serial Display library
 *
 * @Author: Digole Digital Solutions : www.digole.com ported to mbed by Michael Shimniok www.bot-thoughts.com
 */
 
#include "mbed.h"
#include "DigoleSerialDisp.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

char null = 0;


// that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

//UART function

DigoleSerialDisp::DigoleSerialDisp(PinName sda, PinName scl, uint8_t address):
    _device(sda, scl)
{
    _address = (address<<1);
    _device.frequency(100000);
    _Comdelay=70;
}

size_t DigoleSerialDisp::write(const char x)
{
    _device.write(_address, (char *) &x, 1);

    return 1;
}

size_t DigoleSerialDisp::write(const char *str) 
{
    if (str == NULL) return 0;
    return write(str, strlen(str));
}
    
size_t DigoleSerialDisp::write(const char *buffer, size_t size)
{
    int len = 0;
    if (buffer != NULL) {
        _device.write(_address, (char *) buffer, size);
        len = size;
        delay(7);
    }
    return len;
}


size_t DigoleSerialDisp::print(const char c)
{
    buf[0] = 'T';
    buf[1] = 'T';
    buf[2] = c;
    buf[3] = 0;
    write(buf);
    write(null);
    return 1;
}

size_t DigoleSerialDisp::print(const char s[])
{
    int len = strlen(s);

    if (s == NULL) return 0;

    buf[0] = 'T';
    buf[1] = 'T';
    buf[2] = 0;
    strncat(buf, s, 125);
    write(buf);
    write(null);
    return len;
}

size_t DigoleSerialDisp::println(const char s[])
{
    return print(s);
}

/*
 Print.cpp - Base class that provides print() and println()
 Copyright (c) 2008 David A. Mellis.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 Modified 23 November 2006 by David A. Mellis
 */

size_t DigoleSerialDisp::print(unsigned char b, int base)
{
  return print((unsigned long) b, base);
}

size_t DigoleSerialDisp::print(int n, int base)
{
  return print((long) n, base);
}

size_t DigoleSerialDisp::print(unsigned int n, int base)
{
  return print((unsigned long) n, base);
}

size_t DigoleSerialDisp::print(long n, int base)
{
  if (base == 0) {
    return write(n);
  } else if (base == 10) {
    if (n < 0) {
      int t = print('-');
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
  } else {
    return printNumber(n, base);
  }
}

size_t DigoleSerialDisp::print(unsigned long n, int base)
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

size_t DigoleSerialDisp::print(double n, int digits)
{
  return printFloat(n, digits);
}

size_t DigoleSerialDisp::println(unsigned char b, int base)
{
  size_t n = print(b, base);
  n += println();
  return n;
}

size_t DigoleSerialDisp::println(int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t DigoleSerialDisp::println(unsigned int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t DigoleSerialDisp::println(long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t DigoleSerialDisp::println(unsigned long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t DigoleSerialDisp::println(double num, int digits)
{
  size_t n = print(num, digits);
  n += println();
  return n;
}

size_t DigoleSerialDisp::println(void) 
{
    return 1;
}

// Private Methods /////////////////////////////////////////////////////////////

size_t DigoleSerialDisp::printNumber(unsigned long n, uint8_t base) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  return write(str);
}

size_t DigoleSerialDisp::printFloat(double number, uint8_t digits) 
{ 
  size_t n = 0;
  
  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
  if (number <-4294967040.0) return print ("ovf");  // constant determined empirically
  
  // Handle negative numbers
  if (number < 0.0)
  {
     n += print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    n += print("."); 
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    n += print(toPrint);
    remainder -= toPrint; 
  } 
  
  return n;
}

/*---------functions for Text and Graphic LCD adapters---------*/
void DigoleSerialDisp::disableCursor(void) 
{
    write("CS");
    write(null);
}

void DigoleSerialDisp::enableCursor(void) 
{
    write("CS");
    write(1);
}

void DigoleSerialDisp::drawStr(uint8_t x, uint8_t y, const char *s) 
{
    write("TP");
    write(x);
    write(y);
    write("TT");
    write(s);
    write(null);
}

void DigoleSerialDisp::setPrintPos(uint8_t x, uint8_t y, uint8_t graph) 
{
    if (graph == _TEXT_) {
        write("TP");
        write(x);
        write(y);
    } else {
        write("GP");
        write(x);
        write(y);
    }
}

void DigoleSerialDisp::clearScreen(void) 
{
    //write(null);
    write("CL");
}

void DigoleSerialDisp::setLCDColRow(uint8_t col, uint8_t row) 
{
    write("STCR");
    write(col);
    write(row);
    write("\x80\xC0\x94\xD4");
}

void DigoleSerialDisp::setI2CAddress(uint8_t add) 
{
    write("SI2CA");
    write(add);
    _address = (add<<1);
}

void DigoleSerialDisp::displayConfig(uint8_t v) 
{
    write("DC");
    write(v);
}

void DigoleSerialDisp::preprint(void) 
{
    //print("TT");
}

/*----------Functions for Graphic LCD/OLED adapters only---------*/
void DigoleSerialDisp::drawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *bitmap) {
    uint8_t i = 0;
    if ((w & 7) != 0)
        i = 1;
    write("DIM");
    write(x); //x;
    write(y);
    write(w);
    write(h);
    for (int j = 0; j < h * ((w >> 3) + i); j++) {
        write( (const char *) (bitmap+j) );
        delay(1);
    }
}

void DigoleSerialDisp::setRot90(void) {
    write("SD1");
}

void DigoleSerialDisp::setRot180(void) {
    write("SD2");
}

void DigoleSerialDisp::setRot270(void) {
    write("SD3");
}

void DigoleSerialDisp::undoRotation(void) {
    write("SD0");
}

void DigoleSerialDisp::setRotation(uint8_t d) {
    write("SD");
    write(d);
}

void DigoleSerialDisp::setContrast(uint8_t c) {
    write("CT");
    write(c);
}

void DigoleSerialDisp::drawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    write("FR");
    write(x);
    write(y);
    write(x + w);
    write(y + h);
}

void DigoleSerialDisp::drawCircle(uint8_t x, uint8_t y, uint8_t r, uint8_t f) {
    write("CC");
    write(x);
    write(y);
    write(r);
    write(f);
}

void DigoleSerialDisp::drawDisc(uint8_t x, uint8_t y, uint8_t r) {
    drawCircle(x, y, r, 1);
}

void DigoleSerialDisp::drawFrame(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    write("DR");
    write(x);
    write(y);
    write(x + w);
    write(y + h);
}

void DigoleSerialDisp::drawPixel(uint8_t x, uint8_t y, uint8_t color) {
    write("DP");
    write(x);
    write(y);
    write(color);
}

void DigoleSerialDisp::drawLine(uint8_t x, uint8_t y, uint8_t x1, uint8_t y1) {
    write("LN");
    write(x);
    write(y);
    write(x1);
    write(y1);
}

void DigoleSerialDisp::drawLineTo(uint8_t x, uint8_t y) {
    write("LT");
    write(x);
    write(y);
}

void DigoleSerialDisp::drawHLine(uint8_t x, uint8_t y, uint8_t w) {
    drawLine(x, y, x + w, y);
}

void DigoleSerialDisp::drawVLine(uint8_t x, uint8_t y, uint8_t h) {
    drawLine(x, y, x, y + h);
}

void DigoleSerialDisp::nextTextLine(void) {
    write(null);
    write("TRT");
}

void DigoleSerialDisp::setFont(uint8_t font) {
    write("SF");
    write(font);
}

void DigoleSerialDisp::setColor(uint8_t color) {
    write("SC");
    write(color);
}

void DigoleSerialDisp::backLightOn(void) {
    write("BL");
    write(1);
}

void DigoleSerialDisp::backLightOff(void) {
    write("BL");
    write(null);
}

void DigoleSerialDisp::directCommand(uint8_t d) {
    write("MCD");
    write(d);
}

void DigoleSerialDisp::directData(uint8_t d) {
    write("MDT");
    write(d);
}

void DigoleSerialDisp::moveArea(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, char xoffset, char yoffset) {
    write("MA");
    write(x0);
    write(y0);
    write(x1);
    write(y1);
    write(xoffset);
    write(yoffset);
}


void DigoleSerialDisp::displayStartScreen(uint8_t m) {
    write("DSS");
    write(m);
} //display start screen


void DigoleSerialDisp::setMode(uint8_t m) {
    write("DM");
    write(m);
} //set display mode


void DigoleSerialDisp::setTextPosBack(void) {
    write("ETB");
} //set text position back to previous, only one back allowed


void DigoleSerialDisp::setTextPosOffset(char xoffset, char yoffset) {
    write("ETO");
    write(xoffset);
    write(yoffset);
}


void DigoleSerialDisp::setTextPosAbs(uint8_t x, uint8_t y) {
    write("ETP");
    write(x);
    write(y);
}


void DigoleSerialDisp::setLinePattern(uint8_t pattern) {
    write("SLP");
    write(pattern);
}


void DigoleSerialDisp::setLCDChip(uint8_t chip) {      //only for universal LCD adapter
    write("SLCD");
    write(chip);
}


void DigoleSerialDisp::uploadStartScreen(int lon, const unsigned char *data) 
{
    int j;
    uint8_t c;
    write("SSS");
    write((uint8_t) (lon % 256));
    write((uint8_t) (lon / 256));
    for (j = 0; j < lon;j++) {
        if((j%32)==0)
            delay(10);
        delay(_Comdelay);
        c = data[j];
        write(c);
    }
}


void DigoleSerialDisp::uploadUserFont(int lon, const unsigned char *data, uint8_t sect) {
    uint8_t c;
    write("SUF");
    write(sect);
    write((uint8_t) (lon % 256));
    write((uint8_t) (lon / 256));
    for (int j = 0; j < lon; j++) {
        if((j%32)==0)
            delay(10);
        delay(_Comdelay);
        c = data[j];
        write(c);
    }
}

void DigoleSerialDisp::digitalOutput(uint8_t x) 
{
    write("DOUT");
    write(x);
}
