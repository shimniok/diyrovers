#include "Bargraph.h"

#define CHAR_WIDTH 6
#define WIDTH 8
#define HEIGHT 9

SerialGraphicLCD *Bargraph::lcd = 0;

Bargraph::Bargraph(int x, int y, int size, char name):
    _x(x), _y(y), _x2(x+WIDTH), _y2(y+size-1), _s(size), _n(name), _last(0)
{
}

Bargraph::Bargraph(int x, int y, int size, int width, char name):
    _x(x), _y(y), _x2(x+width-1), _y2(y+size-1), _s(size), _w(width), _n(name), _last(0)
{
}

void Bargraph::init() 
{
    if (lcd) {
        if (_n != ' ') {
            lcd->posXY(_x + (_w/2 - CHAR_WIDTH/2), _y2+2); // horizontal center
            //wait_ms(5);
            lcd->printf("%c", _n);
            //wait_ms(5);
        }
        lcd->rect(_x, _y, _x2, _y2, true);
        //wait_ms(5); // doesn't seem to help
        int value = _last;
        _last = 0;
        update(value);
    }
}

void Bargraph::calibrate(float min, float max)
{
    _min = min;
    _max = max;
}

void Bargraph::update(float value)
{
    int ivalue;

    ivalue = (int) ((value - _min) * (_s-1)/(_max - _min));

    update(ivalue);

    return;
}

void Bargraph::update(int value)
{
    if (lcd) {
        if (value >= 0 && value < _s) {
            int newY = _y2-value;
            
            for (int y=_y+1; y < _y2; y++) {
                lcd->line(_x+1, y, _x2-1, y, (y > newY));
                wait_ms(8);
            }
        }
        _last = value;
    }
}

