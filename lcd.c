#include "lcd.h"
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <stdbool.h>
#include "Drivers/driverlib/sysctl.h"
#include "Drivers/driverlib/gpio.h"
#include "Drivers/inc/hw_ints.h"
#include "Drivers/inc/hw_memmap.h"
#include "Drivers/inc/hw_sysctl.h"

#define LCD_PORT GPIO_PORTB_BASE
#define LCD_DATA_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define LCD_RS GPIO_PIN_4
#define LCD_EN GPIO_PIN_5

void lcdPulseEnable() {
    GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN);
    SysCtlDelay(SysCtlClockGet() / (1000 * 3));
    GPIOPinWrite(LCD_PORT, LCD_EN, 0);
    SysCtlDelay(SysCtlClockGet() / (1000 * 3));
}

void lcdWrite4Bits(uint8_t nibble) {
    GPIOPinWrite(LCD_PORT, LCD_DATA_PINS, nibble & 0x0F);
    lcdPulseEnable();
}

void lcdSendCommand(uint8_t cmd) {
    GPIOPinWrite(LCD_PORT, LCD_RS, 0); // RS = 0 for command
    lcdWrite4Bits(cmd >> 4);
    lcdWrite4Bits(cmd & 0x0F);
    SysCtlDelay(SysCtlClockGet() / (3000 * 3));
}

void lcdSendData(uint8_t data) {
    GPIOPinWrite(LCD_PORT, LCD_RS, LCD_RS); // RS = 1 for data
    lcdWrite4Bits(data >> 4);
    lcdWrite4Bits(data & 0x0F);
    SysCtlDelay(SysCtlClockGet() / (3000 * 3));
}

void initLCD() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    GPIOPinTypeGPIOOutput(LCD_PORT, LCD_RS | LCD_EN | LCD_DATA_PINS);
    SysCtlDelay(SysCtlClockGet() / 100); // Wait >15ms

    lcdWrite4Bits(0x03);
    SysCtlDelay(SysCtlClockGet() / 300); // Wait >4.1ms
    lcdWrite4Bits(0x03);
    SysCtlDelay(SysCtlClockGet() / 6000);
    lcdWrite4Bits(0x03);
    lcdWrite4Bits(0x02); // 4-bit mode

    lcdSendCommand(0x28); // 2 lines, 5x8 matrix
    lcdSendCommand(0x0C); // Display on, cursor off
    lcdSendCommand(0x06); // Increment cursor
    lcdSendCommand(0x01); // Clear display
    SysCtlDelay(SysCtlClockGet() / 3000);
}

void LCD_Print(char* str) {
    while (*str) {
        lcdSendData(*str++);
    }
}

void LCD_SetCursor(unsigned char row, unsigned char col) {
    uint8_t address = (row == 0) ? col : (0x40 + col);
    lcdSendCommand(0x80 | address);
}