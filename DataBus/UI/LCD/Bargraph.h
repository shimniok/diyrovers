#ifndef __BARGRAPH_H
#define __BARGRAPH_H

#include "SerialGraphicLCD.h"

class Bargraph {
public:
    Bargraph() {}
    Bargraph(int x, int y, int size, char name);
    Bargraph(int x, int y, int size, int width, char name);
    void init(void);
    void calibrate(float min, float max);
    void update(float value);
    void update(int value);

    static SerialGraphicLCD *lcd;

private:
    int _x;
    int _y;
    int _x2;
    int _y2;
    int _s;
    int _w;
    float _min;
    float _max;
    char _n;
    int _last;
};

#endif