//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include <stdint.h>         // Standard integer types (e.g., uint8_t, uint32_t)
#include <stdbool.h>        // Standard boolean type (true, false)
#include <string.h>         // String manipulation functions (e.g., strlen)
#include "Drivers/inc/hw_memmap.h"  // Macros for memory map registers
#include "Drivers/driverlib/sysctl.h" // System Control library (for SysCtlDelay, SysCtlPeripheralEnable)
#include "Drivers/driverlib/gpio.h"   // GPIO library (for GPIOPinWrite, GPIOPinTypeGPIOOutput)

//*****************************************************************************
//
// LCD Pin Definitions
//
//*****************************************************************************
#define LCD_PORT            GPIO_PORTB_BASE   /**< GPIO port base address for LCD connections (Port B). */
#define LCD_RS              GPIO_PIN_6        /**< GPIO pin for LCD Register Select (RS) - PB6. */
#define LCD_EN              GPIO_PIN_7        /**< GPIO pin for LCD Enable (EN) - PB7. */
// LCD Data pins D4-D7 are connected to PB0-PB3 respectively for 4-bit mode.
#define LCD_DATA_PINS_MASK  (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3) /**< Mask for LCD data pins (PB0-PB3 for D4-D7). */

//*****************************************************************************
//
// Static Helper Functions
//
//*****************************************************************************

/**
 * @brief Provides a blocking delay in milliseconds.
 *
 * Uses the TivaWare SysCtlDelay function. SysCtlDelay takes a number of
 * clock cycles, where each loop is 3 clock cycles.
 *
 * @param ms The number of milliseconds to delay.
 */
static void delayMs(uint32_t ms) {
    // SysCtlClockGet() returns the system clock frequency in Hz (cycles/second).
    // (SysCtlClockGet() / 3000) gives the number of SysCtlDelay loops for 1 ms.
    // (SysCtlClockGet() / (1000 * 3)) where 1000 is for ms and 3 is for SysCtlDelay loop cycles.
    SysCtlDelay((SysCtlClockGet() / 3000) * ms);
}

/**
 * @brief Sends a command to the LCD controller in 4-bit mode.
 *
 * The command is sent as two 4-bit nibbles (upper first, then lower).
 * The RS (Register Select) pin is set to LOW for commands.
 * An enable pulse (EN pin high then low) is generated for each nibble.
 *
 * @param command The 8-bit command to send to the LCD.
 */
static void LCD_SendCommand(uint8_t command) {
    // Set RS to 0 for command mode
    GPIOPinWrite(LCD_PORT, LCD_RS, 0);

    // Send upper nibble (D7-D4)
    // Data pins (PB0-PB3) are mapped to LCD D4-D7.
    // So, (command >> 4) & 0x0F maps the upper 4 bits of the command to PB0-PB3.
    GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, (command >> 4) & 0x0F);
    GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN); // EN high
    delayMs(1);                             // Enable pulse width
    GPIOPinWrite(LCD_PORT, LCD_EN, 0);      // EN low

    // Send lower nibble (D3-D0)
    GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, command & 0x0F);
    GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN); // EN high
    delayMs(1);                             // Enable pulse width
    GPIOPinWrite(LCD_PORT, LCD_EN, 0);      // EN low

    delayMs(2); // Command execution time (varies, 2ms is generally safe for most commands)
}

/**
 * @brief Sends a data byte to the LCD controller in 4-bit mode.
 *
 * The data byte is sent as two 4-bit nibbles (upper first, then lower).
 * The RS (Register Select) pin is set to HIGH for data.
 * An enable pulse (EN pin high then low) is generated for each nibble.
 *
 * @param data The 8-bit data byte to send to the LCD (character to display).
 */
static void LCD_SendData(uint8_t data) {
    // Set RS to 1 for data mode
    GPIOPinWrite(LCD_PORT, LCD_RS, LCD_RS);

    // Send upper nibble (D7-D4)
    GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, (data >> 4) & 0x0F);
    GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN); // EN high
    delayMs(1);                             // Enable pulse width
    GPIOPinWrite(LCD_PORT, LCD_EN, 0);      // EN low

    // Send lower nibble (D3-D0)
    GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, data & 0x0F);
    GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN); // EN high
    delayMs(1);                             // Enable pulse width
    GPIOPinWrite(LCD_PORT, LCD_EN, 0);      // EN low

    delayMs(2); // Data write time (less critical than command time, but delay is good practice)
}

//*****************************************************************************
//
// Public LCD API Functions
//
//*****************************************************************************

/**
 * @brief Initializes the LCD controller for 4-bit operation.
 *
 * Configures GPIO pins for LCD control and data.
 * Sends the required sequence of commands to set up the LCD in 4-bit mode,
 * 2-line display, 5x8 dot characters, display ON, cursor OFF, and auto-increment cursor.
 */
void initLCD(void) {
    // Enable the clock for GPIO Port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Wait for the GPIO Port B peripheral to be ready.
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    // Configure LCD control pins (RS, EN) and data pins (D4-D7 on PB0-PB3) as outputs.
    GPIOPinTypeGPIOOutput(LCD_PORT, LCD_RS | LCD_EN | LCD_DATA_PINS_MASK);

    // Ensure all LCD pins are initially low.
    GPIOPinWrite(LCD_PORT, LCD_RS | LCD_EN | LCD_DATA_PINS_MASK, 0);

    delayMs(20); // Wait for LCD to power up (typically >15ms)

    // LCD Initialization Sequence for 4-bit mode:
    // Refer to HD44780 datasheet for detailed initialization steps.
    // This sequence assumes the LCD is starting from an 8-bit interface state.

    // Step 1: Send 0x30 (Function Set: 8-bit interface) - but only upper nibble matters for wakeup
    // This is often done by sending 0x03 three times with delays for robust initialization.
    // However, a more direct 4-bit init often starts by setting 4-bit mode.
    // The provided code directly sets 4-bit mode.
    // For robustness, one might send 0x03 (upper nibble of 0x30) multiple times before this.
    // Example:
    // GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, 0x03); // Send 0011
    // GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN); delayMs(1); GPIOPinWrite(LCD_PORT, LCD_EN, 0); delayMs(5);
    // GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, 0x03); // Send 0011 again
    // GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN); delayMs(1); GPIOPinWrite(LCD_PORT, LCD_EN, 0); delayMs(1);
    // GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, 0x03); // Send 0011 again
    // GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN); delayMs(1); GPIOPinWrite(LCD_PORT, LCD_EN, 0); delayMs(1);
    // GPIOPinWrite(LCD_PORT, LCD_DATA_PINS_MASK, 0x02); // Set to 4-bit mode (0010)
    // GPIOPinWrite(LCD_PORT, LCD_EN, LCD_EN); delayMs(1); GPIOPinWrite(LCD_PORT, LCD_EN, 0); delayMs(1);

    // Command: Function Set - 4-bit interface, 2 lines, 5x8 dots font.
    // 0x28 = 0010 1000 (DL=0 (4-bit), N=1 (2-line), F=0 (5x8 dots))
    LCD_SendCommand(0x28);

    // Command: Display ON/OFF Control - Display ON, Cursor OFF, Blink OFF.
    // 0x0C = 0000 1100 (D=1 (Display ON), C=0 (Cursor OFF), B=0 (Blink OFF))
    LCD_SendCommand(0x0C);

    // Command: Clear Display - Clears entire display and sets DDRAM address to 0.
    // 0x01 = 0000 0001
    LCD_SendCommand(0x01);
    delayMs(2); // This command takes longer (typically 1.52ms)

    // Command: Entry Mode Set - Increment cursor position, no display shift.
    // 0x06 = 0000 0110 (I/D=1 (Increment), SH=0 (No shift))
    LCD_SendCommand(0x06);
}

/**
 * @brief Prints a null-terminated string to the LCD at the current cursor position.
 *
 * Iterates through the string and sends each character to the LCD as data.
 *
 * @param str Pointer to the null-terminated string to be printed.
 */
void LCD_Print(char* str) {
    if (str == NULL) return; // Safety check for null pointer

    for (size_t i = 0; i < strlen(str); i++) {
        LCD_SendData(str[i]); // Send each character of the string
    }
}

/**
 * @brief Sets the LCD cursor to a specified row and column.
 *
 * Row and column are 0-indexed.
 * For a 16x2 LCD:
 * Row 0: addresses 0x00 to 0x0F
 * Row 1: addresses 0x40 to 0x4F
 * The command to set DDRAM address is 0x80 | address.
 *
 * @param row The row number (0 or 1 for a 2-line display).
 * @param col The column number (0 to 15 for a 16-column display).
 */
void LCD_SetCursor(unsigned char row, unsigned char col) {
    // Basic bounds check for a typical 16x2 LCD
    if (row > 1 || col > 15) return;

    uint8_t address;
    if (row == 0) {
        address = col; // Address for the first line starts at 0x00
    } else { // row == 1
        address = 0x40 + col; // Address for the second line starts at 0x40
    }
    // Send command to set DDRAM address. The command is 0x80 ORed with the address.
    LCD_SendCommand(0x80 | address);
}

/**
 * @brief Clears the entire LCD display and returns the cursor to home (0,0).
 *
 * Sends the "Clear Display" command (0x01) to the LCD.
 * This command also resets any display shift.
 */
void LCD_Clear(void) {
    LCD_SendCommand(0x01); // Send clear display command
    delayMs(2);            // This command requires a longer delay (typically >1.52ms)
}
