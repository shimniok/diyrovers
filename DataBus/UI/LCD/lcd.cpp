#include "mbed.h"
/*
void lcdInit()
{
    lcd.baud(4800);
    lcd.printf("4800");
    lcd.printf("%c%c", 0x7C, 13);
    lcd.baud(9600);
    lcd.printf("9600");
}

void lcdClear()
{
    lcd.printf("%c%c",0xFE,0x01); // command, clear
    wait(0.020);
}    

void lcdSetPos(int x, int y)
{
    uint8_t pos=0;

    if ( x >= 0 && x < 16 && y >= 0 && y < 2)
        pos = x + y*64;
        
    pos |= (1<<7);
    
    lcd.printf("%c%c",0xFE,pos);
}

void lcdSetSplash(const char *s1, const char *s2)
{
    lcdClear();
    lcd.printf(s1);
    lcdSetPos(0,1);
    lcd.printf(s2);
    lcd.printf("%c%c",0x7C,0x0A);
}
*/