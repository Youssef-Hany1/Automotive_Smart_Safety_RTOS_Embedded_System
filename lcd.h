/**
 * @file lcd.h
 * @brief Header file for the HD44780-compatible character LCD driver.
 *
 * This file provides the public interface for initializing and controlling
 * a character LCD display operating in 4-bit mode. It includes functions
 * for printing text, setting the cursor position, and clearing the display.
 *
 * The driver is designed for use with a TM4C123GH6PM microcontroller and
 * assumes specific GPIO pin connections as defined in the corresponding .c file.
 */

#ifndef LCD_H
#define LCD_H

// No specific includes are strictly necessary for the function declarations themselves,
// but <stdint.h> or <stdbool.h> might be included if function parameters used those types.
// In this case, the parameters are basic types (char*, unsigned char).

//*****************************************************************************
//
// Public Function Prototypes
//
//*****************************************************************************

/**
 * @brief Initializes the LCD controller for 4-bit operation.
 *
 * This function must be called before any other LCD operations.
 * It configures the necessary GPIO pins and sends a sequence of commands
 * to the LCD to set it up in 4-bit mode, 2-line display, 5x8 dot characters,
 * with display ON, cursor OFF, and auto-incrementing cursor on writes.
 * Refer to the implementation in lcd.c for specific commands and timings.
 */
void initLCD(void);

/**
 * @brief Prints a null-terminated string to the LCD at the current cursor position.
 *
 * The string is displayed character by character. The cursor will auto-increment
 * to the next position after each character is written, according to the
 * entry mode set during initialization (typically left-to-right).
 *
 * @param str Pointer to the null-terminated C-string to be displayed.
 * If NULL, the function will have no effect (or may cause an error
 * if not handled in the implementation, though the provided .c handles it).
 */
void LCD_Print(char* str);

/**
 * @brief Sets the LCD cursor to a specified row and column position.
 *
 * The display is typically organized with (0,0) being the top-left character.
 * For a 16x2 LCD:
 * - Row 0: Columns 0-15
 * - Row 1: Columns 0-15
 * Attempting to set the cursor outside the valid range may result in
 * undefined behavior or no action, depending on the implementation's bounds checking.
 *
 * @param row The target row number (0-indexed).
 * @param col The target column number (0-indexed).
 */
void LCD_SetCursor(unsigned char row, unsigned char col);

/**
 * @brief Clears the entire LCD display and returns the cursor to the home position (0,0).
 *
 * This operation also clears any previously set display shift.
 * A delay is required after this command for the LCD to process it.
 */
void LCD_Clear(void);

#endif // LCD_H
