#ifndef __GPSSTATUS_H
#define __GPSSTATUS_H

#include "SerialGraphicLCD.h"

class GPSStatus {
public:
    GPSStatus(int x, int y);
    void init();
    void update(float hdop);

    static SerialGraphicLCD *lcd;

private:
    int _x;
    int _y;
    int _s;
    float _min;
    float _max;
    char _n;
    int _last;
};

#endif