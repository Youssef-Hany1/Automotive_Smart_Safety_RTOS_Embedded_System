#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Drivers/inc/hw_memmap.h"
#include "Drivers/driverlib/sysctl.h"
#include "Drivers/driverlib/gpio.h"

#define LCD_PORT GPIO_PORTB_BASE
#define LCD_RS GPIO_PIN_4
#define LCD_EN GPIO_PIN_7
#define LCD_DATA_PINS_MASK (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)

static void delayMs(uint32_t ms) {
    SysCtlDelay((SysCtlClockGet() / 3000) * ms);
}

static void LCD_SendCommand(uint8_t command) {
    GPIOPinWrite(LCD_PORT, LCD_RS, 0);  // RS = 0 for command
    GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, (command >> 4) & 0x0F);  // Upper nibble
    GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN);
    delayMs(1);
    GPIOPinWrite(LCD_PORT, LCD_EN, 0);

    GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, command & 0x0F);  // Lower nibble
    GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN);
    delayMs(1);
    GPIOPinWrite(LCD_PORT, LCD_EN, 0);

    delayMs(2);
}

static void LCD_SendData(uint8_t data) {
    GPIOPinWrite(LCD_PORT, LCD_RS, LCD_RS);  // RS = 1 for data
    GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, (data >> 4) & 0x0F);  // Upper nibble
    GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN);
    delayMs(1);
    GPIOPinWrite(LCD_PORT, LCD_EN, 0);

    GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, data & 0x0F);  // Lower nibble
    GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN);
    delayMs(1);
    GPIOPinWrite(LCD_PORT, LCD_EN, 0);

    delayMs(2);
}

void initLCD(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    GPIOPinTypeGPIOOutput(LCD_PORT, LCD_RS | LCD_EN | LCD_DATA_PINS_MASK);

    // Clear LCD pins
    GPIOPinWrite(LCD_PORT, LCD_RS | LCD_EN | LCD_DATA_PINS_MASK, 0);

    delayMs(20);  // Wait for LCD to power up

    LCD_SendCommand(0x28);  // Function Set: 4-bit, 2-line, 5x8 dots
    LCD_SendCommand(0x0C);  // Display ON, Cursor OFF
    LCD_SendCommand(0x01);  // Clear display
    delayMs(2);             // Wait for clear
    LCD_SendCommand(0x06);  // Entry mode set: Increment cursor
}

void LCD_Print(char* str) {
    for (size_t i = 0; i < strlen(str); i++) {
        LCD_SendData(str[i]);
    }
}

void LCD_SetCursor(unsigned char row, unsigned char col) {
    if (row > 1 || col > 15) return;  // Prevent invalid position

    uint8_t address = (row == 0) ? col : 0x40 + col;
    LCD_SendCommand(0x80 | address);  // Set DDRAM address
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);  // Clear display
    delayMs(2);             // Wait for clear to complete
}

