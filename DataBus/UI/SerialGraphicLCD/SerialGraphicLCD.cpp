#include "SerialGraphicLCD.h"

#define XSIZE 6
#define YSIZE 9

SerialGraphicLCD::SerialGraphicLCD(PinName tx, PinName rx): 
    _lcd(tx, rx), _firmware(SFE_FW)
{
    _lcd.baud(115200);          // default baud rate
    resolution(LCD_128x64);     // default resolution
}

SerialGraphicLCD::SerialGraphicLCD(PinName tx, PinName rx, int firmware):
    _lcd(tx, rx), _firmware(firmware)
{
    _lcd.baud(115200);          // default baud rate
    resolution(LCD_128x64);     // default resolution
}

void SerialGraphicLCD::clear() {
    _lcd.putc((int) 0x7c);
    _lcd.putc(0x00);
}

void SerialGraphicLCD::pos(int col, int row) {
//    if (_firmware == SD_FW)
        posXY(XSIZE*col, (YSIZE*row));
//    else if (_firmware == SFE_FW)
        //posXY(XSIZE*col, _yMax-(YSIZE*row));
}

void SerialGraphicLCD::posXY(int x, int y) {
    _lcd.putc(0x7c);
    _lcd.putc(0x18);
    _lcd.putc(x);
    _lcd.putc(0x7c);
    _lcd.putc(0x19);
    _lcd.putc(y);
}

void SerialGraphicLCD::pixel(int x, int y, const bool set) {
    _lcd.putc(0x7c);
    _lcd.putc(0x10);
    _lcd.putc(x);
    _lcd.putc(y);
    _lcd.putc((set) ? 0x01 : 0x00);
}

void SerialGraphicLCD::line(const int x1, const int y1, const int x2, const int y2, const bool set) {
    _lcd.putc(0x7c);
    _lcd.putc(0x0c);
    _lcd.putc(x1);
    _lcd.putc(y1);
    _lcd.putc(x2);
    _lcd.putc(y2);
    _lcd.putc((set) ? 0x01 : 0x00);
}

void SerialGraphicLCD::circle(const int x, const int y, const int r, const bool set) {
    _lcd.putc(0x7c);
    _lcd.putc(0x03);
    _lcd.putc(x);
    _lcd.putc(y);
    _lcd.putc(r);
    _lcd.putc((set) ? 0x01 : 0x00);
}

// Unfortunately, the datasheet for the stock firmware is incorrect;
// the box command does not take a 5th parameter for draw/erase like the others
// However, it does in the sd firmware
void SerialGraphicLCD::rect(const int x1, const int y1, const int x2, const int y2) {
    _lcd.putc(0x7c);
    _lcd.putc(0x0f);
    _lcd.putc(x1);
    _lcd.putc(y1);
    _lcd.putc(x2);
    _lcd.putc(y2);
    if (_firmware == SD_FW)
        _lcd.putc(0x01);
}

void SerialGraphicLCD::rect(const int x1, const int y1, const int x2, const int y2, const bool set) {
    _lcd.putc(0x7c);
    _lcd.putc(0x0f);
    _lcd.putc(x1);
    _lcd.putc(y1);
    _lcd.putc(x2);
    _lcd.putc(y2);
    if (_firmware == SD_FW)
        _lcd.putc((set) ? 0x01 : 0x00);
}

void SerialGraphicLCD::rectFill(const int x1, const int y1, const int x2, const int y2, char fillByte) {
    if (_firmware == SD_FW) {

        // Bugs in firmware; if y2-y1 == 2, nothing drawn; if y2-y1 == 3, fill is 4 tall
//        if ((y2 - y1) > 3) {
            _lcd.putc(0x7c);
            _lcd.putc(0x12);
            _lcd.putc(x1);
            _lcd.putc(y1);
            _lcd.putc(x2+1); // bug in firmware, off-by-one on x2
            _lcd.putc(y2);
            _lcd.putc(fillByte);
//        } else {
//            for (int y=y1; y <= y2; y++)
//                line(x1, y, x2, y, fillByte == FILL);
//        }            
    }
}

void SerialGraphicLCD::erase(const int x1, const int y1, const int x2, const int y2) {
    _lcd.putc(0x7c);
    _lcd.putc(0x05);
    _lcd.putc(x1);
    _lcd.putc(y1);
    _lcd.putc(x2);
    _lcd.putc(y2);
}

void SerialGraphicLCD::backlight(const int i) {
    if (i >= 0 && i <= 100) {
        _lcd.putc(0x7c);
        _lcd.putc(0x02);
        _lcd.putc(i);
    }
}

void SerialGraphicLCD::reverseMode() {
    _lcd.putc(0x7c);
    _lcd.putc(0x12);
}

void SerialGraphicLCD::resolution(const int type) {
    switch (type) {
    case LCD_128x64 :
        resolution(128, 64);
        break;
    case LCD_160x128 :
        resolution(160, 128);
        break;
    }
}

void SerialGraphicLCD::resolution(const int x, const int y) {
    _xMax = x;
    _yMax = y;
}


void SerialGraphicLCD::lcdbaud(const int b) {
    if (b > 0 && b < 7) {
        _lcd.putc(0x7c);
        _lcd.putc(0x07);
        _lcd.putc(b+'0');
    }
}    

int SerialGraphicLCD::puts(const char *s) {
	return _lcd.puts(s);
}

int SerialGraphicLCD::putc(const char c) {
	return _lcd.putc(c);
}

