#include "GPSStatus.h"

#define WIDTH 8
#define HEIGHT 9

SerialGraphicLCD *GPSStatus::lcd = 0;

GPSStatus::GPSStatus(int x, int y):
    _x(x), _y(y)
{
}

void GPSStatus::init() 
{
    if (lcd) {
        lcd->pixel(_x+1, _y+1, true);
        lcd->pixel(_x+2, _y+1, true);
        lcd->pixel(_x+3, _y+1, true);
        lcd->pixel(_x+2, _y+2, true);
        lcd->pixel(_x+2, _y+3, true);
        lcd->pixel(_x+3, _y+3, true);
        lcd->pixel(_x+4, _y+3, true);
        lcd->pixel(_x+2, _y+4, true);
        lcd->pixel(_x+5, _y+4, true);
        lcd->pixel(_x+1, _y+5, true);
        lcd->pixel(_x+3, _y+5, true);
        lcd->pixel(_x+1, _y+6, true);
        lcd->pixel(_x+2, _y+7, true);
        lcd->posXY(_x+10, _y);
        _last = 0;
    }
}

void GPSStatus::update(float hdop)
{
    lcd->pixel(_x+5, _y+1, hdop < 3.0);    

    lcd->pixel(_x+6, _y+1, hdop < 2.0);    
    lcd->pixel(_x+6, _y+2, hdop < 2.0);    

    lcd->pixel(_x+7, _y+1, hdop < 1.5);    
    lcd->pixel(_x+7, _y+2, hdop < 1.5);    
    lcd->pixel(_x+7, _y+3, hdop < 1.5);    

    lcd->pixel(_x+8, _y+1, hdop < 1.1);    
    lcd->pixel(_x+8, _y+2, hdop < 1.1);    
    lcd->pixel(_x+8, _y+3, hdop < 1.1);    
    lcd->pixel(_x+8, _y+4, hdop < 1.1);    
 
    return;
}