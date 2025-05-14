//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include "FreeRTOS.h"   // FreeRTOS kernel core definitions.
#include <stdint.h>     // Standard integer types (e.g., uint32_t).
#include <stdbool.h>    // Standard boolean type (true, false).
#include "task.h"       // FreeRTOS task management functions (e.g., vTaskStartScheduler).
#include "queue.h"      // FreeRTOS queue management (used by safety_tasks.c).
#include "semphr.h"     // FreeRTOS semaphore management (used by safety_tasks.c).
#include "tm4c123gh6pm.h" // MCU-specific header for TM4C123GH6PM, providing register definitions.
#include "drivers.h"    // Custom peripheral drivers (switches, ADC, ultrasonic, LEDs, buzzer).
#include "lcd.h"        // Custom LCD driver.
#include "safety_tasks.h" // Header for creating the application's safety-related tasks.
#include "Drivers/driverlib/interrupt.h" // TivaWare interrupt control library (for IntMasterEnable).

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

/**
 * @brief Initializes all necessary hardware peripherals for the application.
 *
 * This function calls the initialization routines for various hardware components
 * including switches, LEDs, ADC, ultrasonic sensor, LCD, and buzzer.
 * It should be called once at the beginning of the main function, before
 * starting the FreeRTOS scheduler.
 */
void setupHardware(void);

//*****************************************************************************
//
// Main Function
//
//*****************************************************************************

/**
 * @brief Main entry point of the application.
 *
 * This function performs the following steps:
 * 1. Enables global interrupts for the microcontroller.
 * 2. Calls `setupHardware()` to initialize all required peripherals.
 * 3. Calls `createSafetyTasks()` to create and configure the FreeRTOS tasks
 * that implement the application's core logic.
 * 4. Starts the FreeRTOS scheduler by calling `vTaskStartScheduler()`.
 * 5. Enters an infinite loop (though `vTaskStartScheduler` should not return).
 *
 * @return int This function should ideally not return. A return value of 0
 * indicates nominal termination, but in an embedded FreeRTOS
 * application, control is typically handed over to the scheduler.
 */
int main(void) {
    // Enable global interrupts for the microcontroller.
    // This is crucial for FreeRTOS tick interrupts and peripheral interrupts.
    IntMasterEnable();

    // Initialize all hardware peripherals used by the application.
    setupHardware();

    // Create the FreeRTOS tasks that define the application's behavior.
    // This function is defined in safety_tasks.c (or a similar file).
    createSafetyTasks();

    // Start the FreeRTOS scheduler.
    // This function will start the highest priority task that is ready to run.
    // It should not return unless there is insufficient FreeRTOS heap memory available.
    vTaskStartScheduler();

    // Infinite loop: This part of the code should ideally not be reached if the
    // FreeRTOS scheduler starts successfully. It acts as a fallback in case
    // vTaskStartScheduler() returns (e.g., due to heap allocation issues).
    while (1) {
        // Application should not reach here.
    }
    // The return 0; is conventional for main but typically unreachable in embedded RTOS applications.
}

//*****************************************************************************
//
// Hardware Setup Function
//
//*****************************************************************************

/**
 * @brief Initializes all hardware components required by the application.
 *
 * This function groups all the hardware initialization calls. It ensures that
 * peripherals like switches, LEDs (door lock, RGB), ADC, ultrasonic sensor,
 * LCD, and buzzer are configured and ready for use before the application
 * tasks start operating.
 */
void setupHardware() {
    // Initialize GPIO pins for switches and their interrupt handlers.
    initSwitches();

    // Initialize the GPIO pin for the door lock indicator LED.
    initDoorLockLed();

    // Initialize the Analog-to-Digital Converter (ADC) for speed sensing.
    initADC();

    // Initialize the ultrasonic sensor (trigger pin, echo capture timer).
    initUltrasonic(); // Note: This typically also initializes Timer0 and Timer1.

    // Initialize the LCD display module (GPIO pins, 4-bit mode, display settings).
    initLCD();

    // Initialize the GPIO pin for the buzzer.
    initBuzzer();

    // Initialize the GPIO pins for the RGB LED.
    initRGB();
}
