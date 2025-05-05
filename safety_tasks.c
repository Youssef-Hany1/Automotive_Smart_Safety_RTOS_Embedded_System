#include "FreeRTOS.h"
#include <stdio.h>
#include <stdbool.h>
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lcd.h"
#include "drivers.h"

QueueHandle_t speedQueue;
QueueHandle_t distanceQueue;
SemaphoreHandle_t lcdMutex;
bool doorLocked = false;

// Task prototypes
void readSpeedTask(void* pvParameters);
void gearLogicTask(void* pvParameters);
void doorLockTask(void* pvParameters);
void rearAssistTask(void* pvParameters);
void lcdUpdateTask(void* pvParameters);

void createSafetyTasks() {
    lcdMutex = xSemaphoreCreateMutex();
    speedQueue = xQueueCreate(5, sizeof(int));
    distanceQueue = xQueueCreate(5, sizeof(int));

    xTaskCreate(readSpeedTask, "SpeedRead", 128, NULL, 2, NULL);
    xTaskCreate(gearLogicTask, "Gear", 128, NULL, 2, NULL);
    xTaskCreate(doorLockTask, "DoorLock", 128, NULL, 3, NULL);
    xTaskCreate(rearAssistTask, "Rear", 128, NULL, 2, NULL);
    xTaskCreate(lcdUpdateTask, "LCD", 128, NULL, 1, NULL);
}

// Reads speed from ADC and pushes to queue
void readSpeedTask(void* pvParameters) {
    while (1) {
        int speed = readSpeedADC();
        xQueueSend(speedQueue, &speed, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Checks if ignition is off, and unlocks doors
void gearLogicTask(void* pvParameters) {
    while (1) {
        if (!isIgnitionOn()) {
            unlockDoors();
            doorLocked = false;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Controls automatic and manual door locking
void doorLockTask(void* pvParameters) {
    while (1) {
        int speed;
        if (xQueueReceive(speedQueue, &speed, portMAX_DELAY)) {
            if (isGearDrive() && speed > 10 && !doorLocked) {
                lockDoors();
                doorLocked = true;
            }
        }

        if (isManualLockPressed()) {
            lockDoors();
            doorLocked = true;
        }

        if (isManualUnlockPressed()) {
            unlockDoors();
            doorLocked = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Assists reversing using ultrasonic sensor and buzzer/LEDs
void rearAssistTask(void* pvParameters) {
    while (1) {
        if (isGearReverse()) {
            int dist = measureDistance();
            xQueueSend(distanceQueue, &dist, portMAX_DELAY);

            if (dist < 30) {
                setBuzzerFrequency(1000);
                setRGBColor('R');
            } else if (dist < 100) {
                setBuzzerFrequency(500);
                setRGBColor('Y');
            } else {
                setBuzzerFrequency(0);
                setRGBColor('G');
            }
        } else {
            setBuzzerFrequency(0);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// Updates LCD display with lock status, speed, and distance
void lcdUpdateTask(void* pvParameters) {
    char buffer[17];  // 16 chars + null terminator
    int speed = 0;
    int dist = 0;

    while (1) {
        if (xSemaphoreTake(lcdMutex, portMAX_DELAY)) {
            // Line 0: Door lock status
            LCD_SetCursor(0, 0);
            LCD_Print(doorLocked ? "Doors: LOCKED  " : "Doors: UNLOCKED");

            // Try to read speed and distance
            xQueueReceive(speedQueue, &speed, 0);
            xQueueReceive(distanceQueue, &dist, 0);

            // Line 1: Speed and Distance in compact format
            LCD_SetCursor(1, 0);
            snprintf(buffer, 17, "S:%3dkm D:%3dcm", speed, dist);
            LCD_Print(buffer);

            xSemaphoreGive(lcdMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

