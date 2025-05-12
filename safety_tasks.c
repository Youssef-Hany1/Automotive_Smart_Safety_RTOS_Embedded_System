#include "FreeRTOS.h"
#include <stdio.h>
#include <stdbool.h>
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lcd.h"
#include "drivers.h"
#include "Drivers/driverlib/sysctl.h"
#include "Drivers/driverlib/gpio.h"
#include "Drivers/driverlib/adc.h"
#include "Drivers/driverlib/pin_map.h"
#include "Drivers/driverlib/timer.h"
#include "Drivers/driverlib/interrupt.h"
#include "Drivers/inc/hw_ints.h"
#include "Drivers/inc/hw_memmap.h"

QueueHandle_t speedQueue;
QueueHandle_t distanceQueue;
SemaphoreHandle_t lcdMutex;
bool doorLocked = false;
bool doorOpenedBuzzer = false;
bool distanceBuzzer = false;

// Task prototypes
void doorTask(void* pvParameters);
void rearAssistTask(void* pvParameters);
void lcdUpdateTask(void* pvParameters);

void createSafetyTasks() {
    lcdMutex = xSemaphoreCreateMutex();
    speedQueue = xQueueCreate(1, sizeof(int));
    distanceQueue = xQueueCreate(1, sizeof(uint32_t));

    xTaskCreate(doorTask, "Door", 80, NULL, 3, NULL);  // Reduced priority of doorTask
    xTaskCreate(rearAssistTask, "Rear", 80, NULL, 2, NULL);  // Higher priority for rear assist
    xTaskCreate(lcdUpdateTask, "LCD", 80, NULL, 2, NULL);   // Reduced priority of lcdUpdateTask
}

// Controls automatic and manual door locking
void doorTask(void* pvParameters) {
    int speed;
    static bool wasHigh = false;

    while (1) {
        if (isIgnitionOn()) {
            // 1) Read & publish latest speed
            speed = readSpeedADC();
            xQueueOverwrite(speedQueue, &speed);

            // 2) Auto-lock on motion
            if ((isGearDrive() || isGearReverse()) && speed > 10 && !doorLocked && !isDriverDoorOpen() && !wasHigh) {
                lockDoors();
                setDoorLockLed(1);
                doorLocked = true;
                wasHigh = true;
            }

            if ((isGearDrive() || isGearReverse()) && speed <= 10 && wasHigh) {
                wasHigh = false;
            }

            // 3) Manual-lock lever (PA6 level)
            if (isManualLockOn() && !doorLocked && !isDriverDoorOpen()) {
                lockDoors();
                setDoorLockLed(1);
                doorLocked = true;
            }

            // 4) Manual-unlock lever (PA7 level)
            if (isManualUnlockOn() && doorLocked) {
                unlockDoors();
                setDoorLockLed(0);
                doorLocked = false;
            }

            // 5) Door-status ? buzzer & LCD
            if (isDriverDoorOpen() && !doorLocked) {
                // start beeping
                setOnBuzzer();
								doorOpenedBuzzer = true;
                if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(50))) {
                    LCD_SetCursor(0, 0);
                    LCD_Print("Door Opened     ");
                    xSemaphoreGive(lcdMutex);  // Ensure semaphore is released immediately
                }
            } else {
                // stop beeping
								doorOpenedBuzzer = false;
								if(!distanceBuzzer)
										setOffBuzzer();
            }
        } else {
            // ignition off: ensure doors unlocked & buzzer off
            setOffBuzzer();
            unlockDoors();
            setDoorLockLed(0);
            doorLocked = false;
        }

        // run at ~10 Hz
        vTaskDelay(pdMS_TO_TICKS(50));  // Reduced delay for better responsiveness
    }
}

void rearAssistTask(void* pvParameters) {
    uint32_t dist = 0;

    while (1) {
        if (isIgnitionOn() && isGearReverse()) {
            // Read the distance from the ultrasonic sensor
            dist = ultrasonic_get_distance();
            if (dist > 0) {  // Ensure the distance value is valid
                xQueueOverwrite(distanceQueue, &dist);
                
                // Adjust buzzer frequency and LED color based on distance
                if (dist < 30) {
										if (!doorOpenedBuzzer){
												setBuzzerFrequency(750);   // Close distance
												distanceBuzzer = true;
										}
                    setRGBColor('R');          // Red LED (Danger)
                } else if (dist < 100) {
										if (!doorOpenedBuzzer){
												setBuzzerFrequency(1500);   // Medium distance
												distanceBuzzer = true;
										}   
                    setRGBColor('Y');          // Yellow LED (Warning)
                } else {
										if (!doorOpenedBuzzer){
												setBuzzerFrequency(3000);   // Far distance
												distanceBuzzer = true;
										}     
                    setRGBColor('G');          // Green LED (Safe)
                }
            } else {
                // Handle the case where distance could not be measured
                setOffBuzzer();
								distanceBuzzer = false;
                setRGBColor('0');  // Turn off LEDs
            }
        } else {
            // Turn off buzzer and LEDs when not in reverse & door is not opened
						if (!doorOpenedBuzzer)
								setOffBuzzer();
            setRGBColor('0');       // Turn off LEDs
        }

        // Shortened delay for faster response during reverse
        vTaskDelay(pdMS_TO_TICKS(20));  // Shorter delay for quicker response when reversing
    }
}


// Updates LCD display with lock status, speed, distance, and door status
void lcdUpdateTask(void* pvParameters) {
    char buffer[17];
    int speed = 0, lastSpeed = -1;
    uint32_t dist = 0, lastDist = -1;
    static bool wasOn = false;
    static char lastGear = 'X';     // N, D, R
    static bool lastLock = false;
    static bool doorWasOpen = false; // Track if the door was previously open

    while (1) {
        if (isIgnitionOn()) {
            if (!wasOn) {
                wasOn = true;
                if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(50))) {  // Shortened semaphore wait time
                    LCD_Clear();
                    LCD_SetCursor(0, 0);
                    LCD_Print("Car ON");
                    delay_ms(2000); 
                    LCD_Clear();
                    xSemaphoreGive(lcdMutex);
                }
            }

            char gear = isGearDrive() ? 'D' :
                        isGearReverse() ? 'R' : 'N';
            bool lock = doorLocked;
            bool doorOpen = isDriverDoorOpen();  // Get door status
            xQueueReceive(speedQueue, &speed, 0);
            if (isGearReverse()) {
                xQueueReceive(distanceQueue, &dist, 0);
            }

            if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(50))) {  // Shortened semaphore wait time
                // If the door has been opened, update the screen accordingly
                if (doorOpen && !doorWasOpen) {
                    doorWasOpen = true;
                }

                // If the door has been closed, reprint the gear and lock status
                if (!doorOpen && doorWasOpen && !doorLocked) {
                    LCD_SetCursor(0, 0);
                    sprintf(buffer, "Gear:%c ", gear);
                    LCD_Print(buffer);
                    LCD_SetCursor(0, 7);
                    LCD_Print(lock ? "Doors:L  " : "Doors:UL ");
                    lastGear = gear;   // Update lastGear to avoid redundant prints
                    lastLock = lock;   // Update lastLock to avoid redundant prints
                    doorWasOpen = false;
                } else {
                    if (gear != lastGear || lock != lastLock) {
                        LCD_SetCursor(0, 0);
                        sprintf(buffer, "Gear:%c ", gear);
                        LCD_Print(buffer);
                        LCD_SetCursor(0, 7);
                        LCD_Print(lock ? "Doors:L  " : "Doors:UL ");
                        lastGear = gear;   // Update lastGear to avoid redundant prints
                        lastLock = lock;   // Update lastLock to avoid redundant prints
                    }
                }

                bool needUpdate = false;
                if (gear == 'R' && (speed != lastSpeed || dist != lastDist)) {
                    sprintf(buffer, "S:%3dkm D:%3dcm ", speed, dist);
                    lastSpeed = speed;
                    lastDist = dist;
                    needUpdate = true;
                } else if (gear == 'D' && speed != lastSpeed) {
                    sprintf(buffer, "S:%3dkm         ", speed);
                    lastSpeed = speed;
                    lastDist = -1;
                    needUpdate = true;
                } else if (gear == 'N' && (lastSpeed != -999 || lastDist != -999)) {
                    sprintf(buffer, "                ");
                    lastSpeed = -999;
                    lastDist = -999;
                    needUpdate = true;
                }

                if (needUpdate) {
                    LCD_SetCursor(1, 0);
                    LCD_Print(buffer);
                }

                xSemaphoreGive(lcdMutex);
            }
        } else {
            if (wasOn) {
                wasOn = false;
                if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(50))) {
                    LCD_Clear();
                    LCD_SetCursor(0, 0);
                    LCD_Print("Car OFF");
                    delay_ms(2000);
                    LCD_Clear();
                    xSemaphoreGive(lcdMutex);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Shortened delay for faster updates
    }
}
