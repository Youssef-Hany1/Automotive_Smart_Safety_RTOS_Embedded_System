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

// Task prototypes
void doorTask(void* pvParameters);
void rearAssistTask(void* pvParameters);
void lcdUpdateTask(void* pvParameters);

void createSafetyTasks() {
    lcdMutex = xSemaphoreCreateMutex();
    speedQueue = xQueueCreate(1, sizeof(int));
    distanceQueue = xQueueCreate(1, sizeof(int));

    xTaskCreate(doorTask, "Door", 128, NULL, 3, NULL);
    xTaskCreate(rearAssistTask, "Rear", 128, NULL, 2, NULL);
    xTaskCreate(lcdUpdateTask, "LCD", 128, NULL, 1, NULL);
}

// Controls automatic and manual door locking
void doorTask(void* pvParameters) {
    int speed;

    while (1) {
        if ( isIgnitionOn() ) {
            // 1) Read & publish latest speed
            speed = readSpeedADC();
            xQueueOverwrite(speedQueue, &speed);

            // 2) Auto-lock on motion
            if ((isGearDrive() || isGearReverse()) && speed > 10 && !doorLocked) {
                lockDoors();
                doorLocked = true;
            }

            // 3) Manual-lock lever (PA6 level)
            if ( isManualLockOn() && !doorLocked ) {
                lockDoors();
                doorLocked = true;
            }

            // 4) Manual-unlock lever (PA7 level)
            if ( isManualUnlockOn() && doorLocked ) {
                unlockDoors();
                doorLocked = false;
            }

            // 5) Door-status ? buzzer & LCD
            if ( isDriverDoorOpen() ) {
                // start beeping
                setBuzzerFrequency(125);

                // show “Door is opened!” once
                if ( xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) ) {
                    LCD_Clear();
                    LCD_SetCursor(0, 0);
                    LCD_Print("Door is opened!");
                    xSemaphoreGive(lcdMutex);
                }
            }
            else {
                // stop beeping
                setBuzzerFrequency(0);
            }
        }
        else {
            // ignition off: ensure doors unlocked & buzzer off
            setBuzzerFrequency(0);
            unlockDoors();
            doorLocked = false;

            // clear the display once
            if ( xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100)) ) {
                LCD_Clear();
                xSemaphoreGive(lcdMutex);
            }
        }

        // run at ~10 Hz
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// Assists reversing using ultrasonic sensor and buzzer/LEDs
void rearAssistTask(void* pvParameters) {
    while (1) {
				if (isIgnitionOn()) {
						if (isGearReverse()) {
								int dist = measureDistance();
								xQueueOverwrite(distanceQueue, &dist);

								if (dist < 30) {
										setBuzzerFrequency(125);
										setRGBColor('R');
								} else if (dist < 100) {
										setBuzzerFrequency(250);
										setRGBColor('Y');
								} else {
										setBuzzerFrequency(500);
										setRGBColor('G');
								}
						} else {
								setBuzzerFrequency(0);
						}
				}else{
						LCD_Clear();
				}
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Updates LCD display with lock status, speed, and distance
void lcdUpdateTask(void* pvParameters) {
    char buffer[17];
    int speed = 0, dist = 0;

    while (1) {
        if (isIgnitionOn()) {
            if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100))) {
                // line 0: lock status
                LCD_SetCursor(0, 0);
                LCD_Print(doorLocked ? "Doors: LOCKED  "
                                     : "Doors: UNLOCKED");

                // grab latest speed (always) and distance (we’ll show it only in R)
                xQueueReceive(speedQueue,    &speed, 0);
                xQueueReceive(distanceQueue, &dist,  0);

                // line 1: conditional display
                LCD_SetCursor(1, 0);
                if (isGearReverse()) {
                    // show both speed & distance
                    snprintf(buffer, sizeof(buffer),
                             "S:%3dkm D:%3dcm", speed, dist);
                } else {
                    // only speed; pad out the rest
                    snprintf(buffer, sizeof(buffer),
                             "S:%3dkm        ", speed);
                }
                LCD_Print(buffer);

                xSemaphoreGive(lcdMutex);
            }
        } else {
            // clear once when ignition first goes off
            static bool wasOn = true;
            if (wasOn) {
                if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100))) {
                    LCD_Clear();
                    xSemaphoreGive(lcdMutex);
                }
                wasOn = false;
            }
        }
        // throttle updates to ~1 Hz
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}




