#include "SerialGraphicLCD.h"

#define XSIZE 6
#define YSIZE 9

SerialGraphicLCD::SerialGraphicLCD(PinName tx, PinName rx): 
    Serial(tx, rx), _firmware(SFE_FW)
{
    baud(115200);               // default baud rate
    resolution(LCD_128x64);     // default resolution
}

SerialGraphicLCD::SerialGraphicLCD(PinName tx, PinName rx, int firmware):
    Serial(tx, rx), _firmware(firmware)
{
    baud(115200);               // default baud rate
    resolution(LCD_128x64);     // default resolution
}

void SerialGraphicLCD::clear() {
    putc(0x7c);
    putc(0x00);
}

void SerialGraphicLCD::pos(int col, int row) {
//    if (_firmware == SD_FW)
        posXY(XSIZE*col, (YSIZE*row));
//    else if (_firmware == SFE_FW)
        //posXY(XSIZE*col, _yMax-(YSIZE*row));
}

void SerialGraphicLCD::posXY(int x, int y) {
    putc(0x7c);
    putc(0x18);
    putc(x);
    putc(0x7c);
    putc(0x19);
    putc(y);
}

void SerialGraphicLCD::pixel(int x, int y, bool set) {
    putc(0x7c);
    putc(0x10);
    putc(x);
    putc(y);
    putc((set) ? 0x01 : 0x00);
}

void SerialGraphicLCD::line(int x1, int y1, int x2, int y2, bool set) {
    putc(0x7c);
    putc(0x0c);
    putc(x1);
    putc(y1);
    putc(x2);
    putc(y2);
    putc((set) ? 0x01 : 0x00);
}

void SerialGraphicLCD::circle(int x, int y, int r, bool set) {
    putc(0x7c);
    putc(0x03);
    putc(x);
    putc(y);
    putc(r);
    putc((set) ? 0x01 : 0x00);
}

// Unfortunately, the datasheet for the stock firmware is incorrect;
// the box command does not take a 5th parameter for draw/erase like the others
// However, it does in the sd firmware
void SerialGraphicLCD::rect(int x1, int y1, int x2, int y2) {
    putc(0x7c);
    putc(0x0f);
    putc(x1);
    putc(y1);
    putc(x2);
    putc(y2);
    if (_firmware == SD_FW)
        putc(0x01);
}

void SerialGraphicLCD::rect(int x1, int y1, int x2, int y2, bool set) {
    putc(0x7c);
    putc(0x0f);
    putc(x1);
    putc(y1);
    putc(x2);
    putc(y2);
    if (_firmware == SD_FW)
        putc((set) ? 0x01 : 0x00);
}

void SerialGraphicLCD::rectFill(int x1, int y1, int x2, int y2, char fillByte) {
    if (_firmware == SD_FW) {

        // Bugs in firmware; if y2-y1 == 2, nothing drawn; if y2-y1 == 3, fill is 4 tall
//        if ((y2 - y1) > 3) {
            putc(0x7c);
            putc(0x12);
            putc(x1);
            putc(y1);
            putc(x2+1); // bug in firmware, off-by-one on x2
            putc(y2);
            putc(fillByte);
//        } else {
//            for (int y=y1; y <= y2; y++)
//                line(x1, y, x2, y, fillByte == FILL);
//        }            
    }
}

void SerialGraphicLCD::erase(int x1, int y1, int x2, int y2) {
    putc(0x7c);
    putc(0x05);
    putc(x1);
    putc(y1);
    putc(x2);
    putc(y2);
}

void SerialGraphicLCD::backlight(int i) {
    if (i >= 0 && i <= 100) {
        putc(0x7c);
        putc(0x02);
        putc(i);
    }
}

void SerialGraphicLCD::reverseMode() {
    putc(0x7c);
    putc(0x12);
}

void SerialGraphicLCD::resolution(int type) {
    switch (type) {
    case LCD_128x64 :
        resolution(128, 64);
        break;
    case LCD_160x128 :
        resolution(160, 128);
        break;
    }
}

void SerialGraphicLCD::resolution(int x, int y) {
    _xMax = x;
    _yMax = y;
}


void SerialGraphicLCD::lcdbaud(int b) {
    if (b > 0 && b < 7) {
        putc(0x7c);
        putc(0x07);
        putc(b+'0');
    }
}    
