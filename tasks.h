#ifndef TASKS_H
#define TASKS_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h" // Includes base types like BaseType_t
#include "task.h"     // For TaskHandle_t
#include "queue.h"    // For QueueHandle_t
#include "semphr.h"   // For SemaphoreHandle_t

// --- Shared Data Structures and Enums ---

// Gear States
typedef enum {
    PARK,
    REVERSE,
    DRIVE,
    GEAR_UNKNOWN // Optional default/error state
} GearState;

// Structure for system status updates via queue (example)
typedef struct {
    char statusMessage[20]; // Short status message
    int value;              // Optional associated value
} SystemStatusUpdate;


// --- Task Function Prototypes ---
void vReadSpeedTask(void *pvParameters);             // Reads potentiometer (speed)
void vReadGearTask(void *pvParameters);              // Reads gear selector switches
void vReadDoorStatusTask(void *pvParameters);        // Reads driver door switch
void vReadManualControlsTask(void *pvParameters);    // Reads manual lock/unlock buttons (or handles ISR flags)
void vReadIgnitionTask(void *pvParameters);          // Reads ignition switch
void vDoorLockControlTask(void *pvParameters);       // Controls door lock mechanism based on inputs
void vReadUltrasonicTask(void *pvParameters);        // Triggers and reads ultrasonic sensor
void vParkingAssistFeedbackTask(void *pvParameters); // Controls buzzer and RGB LED for parking assist
void vUpdateDisplayTask(void *pvParameters);         // Updates the LCD display

// --- ISR Handler Prototype ---
// Defined in main.c but potentially called by hardware ISR wrappers
void ISRHandlers(void);

#endif // TASKS_H
