#ifndef LCD_H
#define LCD_H

void initLCD();
void LCD_Print(char* str);
void LCD_SetCursor(unsigned char row, unsigned char col);
void LCD_Clear(void);

#endif