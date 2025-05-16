/**
 * @file drivers.h
 * @brief Header file for peripheral drivers for the TM4C123GH6PM microcontroller.
 *
 * This file contains function declarations, type definitions, and external variable
 * declarations for controlling various hardware peripherals including GPIO switches,
 * ADC for speed sensing, ultrasonic distance sensor, buzzer, RGB LED, and door lock LED.
 * It also defines system state checking functions.
 */

#ifndef DRIVERS_H
#define DRIVERS_H

#include <stdint.h>  // Standard integer types (e.g., uint32_t)
#include <stdbool.h> // Standard boolean type (true, false)

//*****************************************************************************
//
// Type Definitions
//
//*****************************************************************************

/**
 * @brief Enumeration defining the possible gear states of the vehicle.
 */
typedef enum {
    GEAR_NEUTRAL = 0, /**< Gear is in Neutral */
    GEAR_DRIVE   = 1, /**< Gear is in Drive */
    GEAR_REVERSE = 2  /**< Gear is in Reverse */
} Gear_t;

//*****************************************************************************
//
// External Global Variables (defined in drivers.c)
//
//*****************************************************************************

/**
 * @brief Current gear state of the vehicle.
 * Updated by the GPIO interrupt handler based on gear lever inputs (PA2, PA3).
 * Marked volatile as it can be changed by an ISR.
 */
extern volatile Gear_t gearState;

/**
 * @brief State of the manual lock lever (e.g., PA6).
 * True indicates the lever is in the 'lock' position.
 * Updated by the GPIO interrupt handler.
 * Marked volatile as it can be changed by an ISR.
 */
extern volatile bool manualLockState;

/**
 * @brief State of the manual unlock lever (e.g., PA7).
 * True indicates the lever is in the 'unlock' position.
 * Updated by the GPIO interrupt handler.
 * Marked volatile as it can be changed by an ISR.
 */
extern volatile bool manualUnlockState;

// Note: ignitionState and driverDoorState are also volatile globals
// defined in drivers.c but accessed via functions like isIgnitionOn().

//*****************************************************************************
//
// Delay Functions
//
//*****************************************************************************

/**
 * @brief Provides a blocking delay in milliseconds.
 *
 * Accuracy depends on the system clock frequency and calibration of the inner loop.
 * Uses a simple busy-wait loop.
 *
 * @param ms The number of milliseconds to delay.
 */
void delay_ms(uint32_t ms);

//*****************************************************************************
//
// Initialization Functions
//
//*****************************************************************************

/**
 * @brief Initializes GPIO Port A pins (PA2-PA7) for switch inputs.
 * Configures pins as inputs with pull-ups and sets up edge-triggered interrupts.
 */
void initSwitches(void);

/**
 * @brief Initializes ADC0 Sequence 3 to read from AIN0 (PE3).
 * Configures the ADC for processor-triggered, single-channel conversion.
 */
void initADC(void);

/**
 * @brief Initializes the ultrasonic sensor system.
 * Configures the trigger pin (output), echo pin (input capture via Timer1),
 * and a timer (Timer0) for the trigger pulse delay.
 */
void initUltrasonic(void);

/**
 * @brief Initializes the buzzer control pin (e.g., PF3) as a GPIO output.
 */
void initBuzzer(void);

/**
 * @brief Initializes the RGB LED control pins (e.g., PF1, PF2) as GPIO outputs.
 */
void initRGB(void);

/**
 * @brief Initializes the Door Lock indicator LED pin (e.g., PF4) as a GPIO output.
 */
void initDoorLockLed(void);

//*****************************************************************************
//
// Sensor Reading Functions
//
//*****************************************************************************

/**
 * @brief Reads the analog value from the configured ADC channel (PE3).
 * Performs an ADC conversion and scales the result to represent speed.
 *
 * @return int The calculated speed (e.g., in km/h, based on scaling).
 */
int readSpeedADC(void);

/**
 * @brief Measures distance using the ultrasonic sensor.
 * Triggers the sensor, captures the echo pulse using Timer1, and calculates
 * the distance based on the pulse duration.
 *
 * @return uint32_t The measured distance in centimeters (capped at a max value).
 */
uint32_t ultrasonic_get_distance(void);

/**
 * @brief Interrupt Service Routine handler for Timer1A (Ultrasonic Echo Capture).
 * This function should be registered as the ISR for Timer1A capture events.
 * It reads the timer values on rising/falling edges of the echo pulse.
 * Note: While declared here for completeness if needed externally, ISRs are often
 * kept static within the .c file unless required elsewhere.
 */
void ultrasonic_edge_processor(void);

//*****************************************************************************
//
// Actuator Control Functions
//
//*****************************************************************************

/**
 * @brief Turns the buzzer ON by setting its control pin HIGH.
 */
void setOnBuzzer(void);

/**
 * @brief Turns the buzzer OFF by setting its control pin LOW.
 */
void setOffBuzzer(void);

/**
 * @brief Toggles the buzzer to create a tone (square wave).
 * Turns the buzzer ON, delays, turns it OFF, delays.
 *
 * @param frequency_ms_half_period The duration (ms) for the ON and OFF phases (half period).
 */
void setBuzzerFrequency(int frequency_ms_half_period);

/**
 * @brief Sets the color of the RGB LED.
 *
 * @param color Character representing the desired color:
 * 'R' = Red, 'G' = Green (or Blue on LaunchPad), 'Y' = Yellow (R+G/B),
 * other = Off. Behavior depends on specific pin connections (PF1, PF2, PF3).
 */
void setRGBColor(char color);

/**
 * @brief Sets the state of the Door Lock indicator LED.
 *
 * @param status 1 to turn the LED ON, 0 to turn the LED OFF.
 */
void setDoorLockLed(int status);

//*****************************************************************************
//
// System State Checking Functions
//
//*****************************************************************************

/**
 * @brief Checks if the current gear state is DRIVE.
 * Reads the global volatile `gearState` variable.
 * @return True if gearState is GEAR_DRIVE, false otherwise.
 */
bool isGearDrive(void);

/**
 * @brief Checks if the current gear state is REVERSE.
 * Reads the global volatile `gearState` variable.
 * @return True if gearState is GEAR_REVERSE, false otherwise.
 */
bool isGearReverse(void);

/**
 * @brief Checks if the ignition switch is currently ON.
 * Reads the internal ignition state variable (updated by ISR).
 * @return True if ignition is ON, false otherwise.
 */
bool isIgnitionOn(void);

/**
 * @brief Checks if the driver's door switch indicates the door is open.
 * Reads the internal door state variable (updated by ISR).
 * @return True if driver's door is OPEN, false otherwise.
 */
bool isDriverDoorOpen(void);

//*****************************************************************************
//
// Manual Input State Functions
//
//*****************************************************************************

/**
 * @brief Checks the current position of the manual lock lever.
 * Reads the global volatile `manualLockState` variable (updated by ISR).
 * @return True if the lever is currently in the 'lock' position, false otherwise.
 */
bool isManualLockOn(void);

/**
 * @brief Checks the current position of the manual unlock lever.
 * Reads the global volatile `manualUnlockState` variable (updated by ISR).
 * @return True if the lever is currently in the 'unlock' position, false otherwise.
 */
bool isManualUnlockOn(void);


#endif // DRIVERS_H
