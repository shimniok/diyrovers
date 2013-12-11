#ifndef __Display_H
#define __Display_H

#include "SystemState.h"
#include "SerialGraphicLCD.h"
#include "Bargraph.h"

class Display {
public:

    /** create new display instance
     *
     */
    Display(void);

    /** initialize the display
     *
     */
    void init(void);

    /** display a status string
     * @param st is the status string to display
     */
    void status(const char *st);

    /** display a menu item
     * @param itemName is the itemName to display
     */
    void menu(const char *itemName);

    /** display a selected menu item
     * @param itemName is the itemName to display
     */
    void select(const char *itemName);

    /** display gauge at a given position (slot) along the bottom
     */
    void gauge(int slot);

    /** updates data on the display */
    void update(SystemState state);
    
    /** initializes the update display */
    void redraw(void);

    SerialGraphicLCD lcd;
    
private:
    Bargraph v;
    Bargraph a;
    Bargraph g1;
    Bargraph g2;
};        

#endif