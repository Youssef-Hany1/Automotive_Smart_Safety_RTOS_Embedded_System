#include "FreeRTOS.h"
#include <stdio.h>
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lcd.h"
#include "drivers.h"

QueueHandle_t speedQueue;
QueueHandle_t distanceQueue;
SemaphoreHandle_t lcdMutex;
int doorLocked = 0;

void readSpeedTask(void* pvParameters);
void gearLogicTask(void* pvParameters);
void doorLockTask(void* pvParameters);
void rearAssistTask(void* pvParameters);
void lcdUpdateTask(void* pvParameters);

void createSafetyTasks() {
    lcdMutex = xSemaphoreCreateMutex();
    speedQueue = xQueueCreate(5, sizeof(int));
    distanceQueue = xQueueCreate(5, sizeof(int));

    xTaskCreate(readSpeedTask, "SpeedRead", 100, NULL, 2, NULL);
    xTaskCreate(gearLogicTask, "Gear", 100, NULL, 2, NULL);
    xTaskCreate(doorLockTask, "DoorLock", 100, NULL, 3, NULL);
    xTaskCreate(rearAssistTask, "Rear", 100, NULL, 2, NULL);
    xTaskCreate(lcdUpdateTask, "LCD", 100, NULL, 1, NULL);
}

void readSpeedTask(void* pvParameters) {
    while (1) {
        int speed = readSpeedADC();
        xQueueSend(speedQueue, &speed, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void gearLogicTask(void* pvParameters) {
    while (1) {
        if (!isIgnitionOn()) {
            unlockDoors();
            doorLocked = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void doorLockTask(void* pvParameters) {
    while (1) {
        int speed;
        if (xQueueReceive(speedQueue, &speed, portMAX_DELAY)) {
            if (isGearDrive() && speed > 10 && !doorLocked) {
                lockDoors();
                doorLocked = 1;
            }
        }

        if (isManualLockPressed()) {
            lockDoors();
            doorLocked = 1;
        }
        if (isManualUnlockPressed()) {
            unlockDoors();
            doorLocked = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

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

void lcdUpdateTask(void* pvParameters) {
    char buffer[32];
    int speed = 0;
    int dist = 0;
    while (1) {
        if (xSemaphoreTake(lcdMutex, portMAX_DELAY)) {
            LCD_SetCursor(0, 0);
            if (doorLocked) LCD_Print("Doors: LOCKED  ");
            else LCD_Print("Doors: UNLOCKED");

            if (xQueueReceive(speedQueue, &speed, 0)) {
                LCD_SetCursor(1, 0);
                sprintf(buffer, "Speed: %d km/h ", speed);
                LCD_Print(buffer);
            }
            if (xQueueReceive(distanceQueue, &dist, 0)) {
                LCD_SetCursor(1, 15);
                sprintf(buffer, "Dist: %d cm", dist);
                LCD_Print(buffer);
            }
            xSemaphoreGive(lcdMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}