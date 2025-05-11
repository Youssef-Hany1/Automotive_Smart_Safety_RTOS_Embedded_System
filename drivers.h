#ifndef DRIVERS_H
#define DRIVERS_H

#include <stdint.h>
#include <stdbool.h>

void delay_ms(uint32_t ms);

// Initialization functions
void initSwitches(void);         // Initialize Switches port and interrupts
void initADC(void);              // Initialize ADC for speed sensing
void initUltrasonic(void);       // Initialize ultrasonic sensor and timer
void initBuzzer(void);           // Initialize buzzer pin
void initRGB(void);              // Initialize RGB LED
void initDoorLockLed(void);      // Initialize DoorLockedLed LED

// Sensor reading functions
int readSpeedADC(void);          // Read speed from ADC (returns speed in km/h)
int measureDistance(void);       // Measure distance using ultrasonic (returns cm)

// Actuator control functions
void setBuzzerFrequency(int frequency); // Control buzzer (simple on/off)
void setRGBColor(char color);           // Set RGB LED color ('R', 'G', 'Y')
void setDoorLockLed(int status);       // Set DoorLockLed

// Gear & vehicle state functions
bool isGearDrive(void);          // Returns true if gear is in Drive
bool isGearReverse(void);        // Returns true if gear is in Reverse
bool isIgnitionOn(void);         // Returns true if ignition is ON
bool isDriverDoorOpen(void);     // Returns true if driver's door is open

// Lock control functions
void lockDoors(void);            // Engage the door lock (red LED on)
void unlockDoors(void);          // Disengage the door lock (red LED off)

// Manual input flags
bool isManualLockPressed(void);   // Returns true if manual lock was triggered
bool isManualUnlockPressed(void); // Returns true if manual unlock was triggered

// a little enum to track gear
typedef enum { GEAR_NEUTRAL=0, GEAR_DRIVE=1, GEAR_REVERSE=2 } Gear_t;

// the current gear state (0=none/neutral,1=D,2=R)
extern volatile Gear_t gearState;

// Manual-lock/unlock lever state (level-driven)
extern volatile bool manualLockState;    // true when PA6 is “up”
extern volatile bool manualUnlockState;  // true when PA7 is “up”

// Query the current lever position
bool isManualLockOn(void);    // true while the “lock” lever is up
bool isManualUnlockOn(void);  // true while the “unlock” lever is up

#endif // DRIVERS_H
