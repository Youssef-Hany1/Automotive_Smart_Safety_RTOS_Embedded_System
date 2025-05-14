/* ===========================================================================
 * File: main.c
 *
 * Description:
 *   Entry point and hardware setup for the vehicle safety system.
 *   This module initializes all peripherals and starts the FreeRTOS scheduler.
 * ===========================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"                // FreeRTOS kernel
#include <stdint.h>                   // Standard integer types
#include <stdbool.h>                  // Standard boolean types
#include "task.h"                    // FreeRTOS task management
#include "queue.h"                   // FreeRTOS queue management
#include "semphr.h"                  // FreeRTOS semaphores
#include "tm4c123gh6pm.h"            // MCU register definitions
#include "drivers.h"                 // Hardware driver interfaces
#include "lcd.h"                     // LCD display interface
#include "safety_tasks.h"            // Safety-related FreeRTOS tasks
#include "Drivers/driverlib/interrupt.h" // Interrupt control

/* Function Prototypes -------------------------------------------------------*/
/**
 * @brief  Initialize system hardware (GPIOs, ADC, Timers, etc.).
 */
void setupHardware(void);

/* Main Function -------------------------------------------------------------*/
/**
 * @brief  Application entry point.
 * @retval int (never returns under correct operation)
 */
int main(void) {
    IntMasterEnable();      /* Enable global interrupts */
    setupHardware();        /* Configure all hardware peripherals */
    createSafetyTasks();    /* Create FreeRTOS safety monitoring tasks */
    vTaskStartScheduler();  /* Start FreeRTOS scheduler; tasks now running */

    /* Infinite loop: should never reach here if scheduler starts correctly */
    while (1) {
    }
    return 0;               /* Standard return (not reached) */
}

/* Hardware Initialization ---------------------------------------------------*/
/**
 * @brief  Initialize all hardware modules required by the system.
 *         - Switch inputs (gear, lock/unlock, door)
 *         - Door-lock LED indicator
 *         - ADC for speed measurement
 *         - Ultrasonic sensor for rear assist
 *         - Character LCD display
 *         - Buzzer for audible alerts
 *         - RGB LED status indicator
 */
void setupHardware(void) {
    initSwitches();         /* Configure input switches and interrupts */
    initDoorLockLed();      /* Configure door lock LED output */
    initADC();              /* Initialize ADC for speed sensor */
    initUltrasonic();       /* Initialize ultrasonic sensor/timer */
    initLCD();              /* Initialize 16x2 LCD display */
    initBuzzer();           /* Initialize buzzer output pin */
    initRGB();              /* Initialize RGB LED outputs */
}
