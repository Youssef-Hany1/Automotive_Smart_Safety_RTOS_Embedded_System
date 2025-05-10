#include "FreeRTOS.h"
#include <stdint.h>
#include <stdbool.h>
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tm4c123gh6pm.h"
#include "drivers.h"
#include "lcd.h"
#include "safety_tasks.h"

void setupHardware();

int main(void) {
    setupHardware();
    createSafetyTasks();
    vTaskStartScheduler();
    while (1);
}

void setupHardware() {
    initGPIO();
    initADC();
    initUltrasonic();
    initLCD();
    initBuzzer();
    initRGB();
}
