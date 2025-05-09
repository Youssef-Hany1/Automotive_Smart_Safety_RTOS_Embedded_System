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
    speedQueue = xQueueCreate(5, sizeof(int));
    distanceQueue = xQueueCreate(5, sizeof(int));

    xTaskCreate(doorTask, "Door", 128, NULL, 3, NULL);
    xTaskCreate(rearAssistTask, "Rear", 128, NULL, 2, NULL);
    xTaskCreate(lcdUpdateTask, "LCD", 128, NULL, 1, NULL);
}

// Controls automatic and manual door locking
void doorTask(void* pvParameters) {
    while (1) {
				if (isIgnitionOn()) {
						int speed = readSpeedADC();
						xQueueSend(speedQueue, &speed, portMAX_DELAY);
					
						if ((isGearDrive()||isGearReverse()) && speed > 10 && !doorLocked) {
								lockDoors();
								doorLocked = true;
						}

						if (isManualLockPressed()) {
								lockDoors();
								doorLocked = true;
						}

						if (isManualUnlockPressed()) {
								unlockDoors();
								doorLocked = false;
						}
						
						if(isDriverDoorOpen()){
								if (xSemaphoreTake(lcdMutex, portMAX_DELAY)) {
										LCD_Clear();
									
										//Door status
										LCD_SetCursor(0, 0);
										
										LCD_Print("Door is opened!");
										
										GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
										
										xSemaphoreGive(lcdMutex);
								}
						}
				}else{
						LCD_Clear();
						unlockDoors();
            doorLocked = false;
				}
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Assists reversing using ultrasonic sensor and buzzer/LEDs
void rearAssistTask(void* pvParameters) {
    while (1) {
				if (isIgnitionOn()) {
						if (isGearReverse()) {
								int dist = measureDistance();
								xQueueSend(distanceQueue, &dist, portMAX_DELAY);

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
    char buffer[17];  // 16 chars + null terminator
    int speed = 0;
    int dist = 0;

    while (1) {
				if (isIgnitionOn()) {
						if (xSemaphoreTake(lcdMutex, portMAX_DELAY)) {
								LCD_Clear();
							
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
				}else{
						LCD_Clear();
				}
    }
}

