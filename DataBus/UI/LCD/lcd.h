#ifndef __LCD_H
#define __LCD_H

void lcdSetSplash(const char *s1, const char *s2);
void lcdInit(void);
void lcdClear(void);
void lcdSetPos(int x, int y);

extern Serial lcd;

#endif