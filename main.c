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
#include "Drivers/driverlib/interrupt.h"


void setupHardware();

int main(void) {
	  IntMasterEnable();  // Enable global interrupts
    setupHardware();
    createSafetyTasks();
    vTaskStartScheduler();
    while (1);
}

void setupHardware() {
    initSwitches();
		initDoorLockLed();
    initADC();
    ultrasonic_init();
    initLCD();
    initBuzzer();
    initRGB();
}
