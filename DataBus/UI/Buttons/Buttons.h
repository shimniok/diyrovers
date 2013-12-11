#ifndef __BUTTONS_H
#define __BUTTONS_H

#define NEXT_BUTTON   1
#define PREV_BUTTON   2
#define SELECT_BUTTON 3

class Buttons {
public:
    Buttons();
    void init(void);
    void nextPressed(void);
    void prevPressed(void);
    void selectPressed(void);
    
    int which;
    bool pressed;
};

#endif