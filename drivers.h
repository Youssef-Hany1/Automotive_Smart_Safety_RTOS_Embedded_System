#ifndef DRIVERS_H
#define DRIVERS_H

#include <stdint.h>
#include <stdbool.h>

// Initialization functions
void initGPIO(void);             // Initialize all GPIO ports and interrupts
void initADC(void);              // Initialize ADC for speed sensing
void initUltrasonic(void);       // Initialize ultrasonic sensor and timer
void initBuzzer(void);           // Initialize buzzer pin
void initRGB(void);              // Initialize RGB LED

// Sensor reading functions
int readSpeedADC(void);          // Read speed from ADC (returns speed in km/h)
int measureDistance(void);       // Measure distance using ultrasonic (returns cm)

// Actuator control functions
void setBuzzerFrequency(int frequency); // Control buzzer (simple on/off)
void setRGBColor(char color);           // Set RGB LED color ('R', 'G', 'Y', 'N')

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

#endif // DRIVERS_H
