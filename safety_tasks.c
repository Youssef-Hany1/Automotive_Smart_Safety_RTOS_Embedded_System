//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include "FreeRTOS.h"   // FreeRTOS kernel core
#include <stdio.h>      // Standard I/O functions (for sprintf)
#include <stdbool.h>    // Standard boolean type (true, false)
#include "task.h"       // FreeRTOS task management
#include "queue.h"      // FreeRTOS queue management
#include "semphr.h"     // FreeRTOS semaphore management
#include "lcd.h"        // LCD driver
#include "drivers.h"    // Custom peripheral drivers (from previous context)

// TivaWare DriverLib Includes
#include "Drivers/driverlib/sysctl.h"   // System Control library
#include "Drivers/driverlib/gpio.h"     // GPIO library
#include "Drivers/driverlib/adc.h"      // ADC library
#include "Drivers/driverlib/pin_map.h"  // Pin mapping
#include "Drivers/driverlib/timer.h"    // Timer library
#include "Drivers/driverlib/interrupt.h"// Interrupt library
#include "Drivers/inc/hw_ints.h"        // Hardware interrupt numbers
#include "Drivers/inc/hw_memmap.h"      // Hardware memory map

//*****************************************************************************
//
// Global Variables and RTOS Handles
//
//*****************************************************************************

// Queues for inter-task communication
QueueHandle_t speedQueue;    /**< Queue to send speed data from doorTask to lcdUpdateTask. Holds one int. */
QueueHandle_t distanceQueue; /**< Queue to send ultrasonic distance data from rearAssistTask to lcdUpdateTask. Holds one uint32_t. */

// Semaphore for mutual exclusion
SemaphoreHandle_t lcdMutex;  /**< Mutex to protect shared access to the LCD. */

// Global state flags
bool doorLocked = false;       /**< True if doors are currently locked, false otherwise. Shared between tasks. */
bool doorOpenedBuzzer = false; /**< True if the door open buzzer is currently active. Used to coordinate buzzer control. */
bool distanceBuzzer = false;   /**< True if the rear assist distance buzzer is currently active. Used to coordinate buzzer control. */

//*****************************************************************************
//
// Task Prototypes
//
//*****************************************************************************

/**
 * @brief Task responsible for door locking logic, speed monitoring, and driver door status.
 * @param pvParameters Unused.
 */
void doorTask(void* pvParameters);

/**
 * @brief Task responsible for rear parking assistance using the ultrasonic sensor.
 * @param pvParameters Unused.
 */
void rearAssistTask(void* pvParameters);

/**
 * @brief Task responsible for updating the LCD display with system information.
 * @param pvParameters Unused.
 */
void lcdUpdateTask(void* pvParameters);

//*****************************************************************************
//
// Task Creation Function
//
//*****************************************************************************

/**
 * @brief Creates and initializes FreeRTOS tasks and synchronization primitives.
 *
 * This function initializes a mutex for LCD access and two queues for passing
 * speed and distance data between tasks. It then creates the three main tasks:
 * doorTask, rearAssistTask, and lcdUpdateTask, assigning them stack sizes and priorities.
 */
void createSafetyTasks() {
    // Create a mutex for protecting LCD access.
    lcdMutex = xSemaphoreCreateMutex();

    // Create a queue to hold one integer value for speed.
    // Data is overwritten if the queue is full.
    speedQueue = xQueueCreate(1, sizeof(int));

    // Create a queue to hold one uint32_t value for distance.
    // Data is overwritten if the queue is full.
    distanceQueue = xQueueCreate(1, sizeof(uint32_t));

    // Create the door management task.
    // - "Door": Task name for debugging.
    // - 80: Stack size in words (adjust as needed).
    // - NULL: Task parameters (none).
    // - 3: Task priority (higher number = higher priority). Reduced from original.
    // - NULL: Task handle (not stored).
    xTaskCreate(doorTask, "Door", 80, NULL, 3, NULL);

    // Create the rear assistance task.
    // - Priority 2, making it potentially preempt doorTask if both are ready and doorTask is lower.
    xTaskCreate(rearAssistTask, "Rear", 80, NULL, 2, NULL);

    // Create the LCD update task.
    // - Priority 2.
    xTaskCreate(lcdUpdateTask, "LCD", 80, NULL, 2, NULL);
}

//*****************************************************************************
//
// Task Definitions
//
//*****************************************************************************

/**
 * @brief Task to manage door locking, speed sensing, and door status warnings.
 *
 * This task performs the following functions:
 * 1. Reads the current speed using ADC and sends it to `speedQueue`.
 * 2. Automatically locks the doors if the car is in Drive or Reverse, speed is > 10,
 * doors are not already locked, and the driver's door is closed.
 * 3. Handles manual lock/unlock requests from levers (PA6, PA7).
 * 4. Activates a buzzer and displays "Door Opened" on LCD if the driver's door is
 * opened while the doors are unlocked and ignition is on.
 * 5. Ensures doors are unlocked and buzzer is off if ignition is off.
 *
 * @param pvParameters Unused.
 */
void doorTask(void* pvParameters) {
    int speed;                     // Variable to store current speed.
    static bool wasHigh = false;   // Flag to track if speed was previously > 10 km/h for auto-lock hysteresis.

    while (1) {
        if (isIgnitionOn()) {
            // 1) Read & publish latest speed
            speed = readSpeedADC(); // Read speed from ADC driver function.
            // Send speed to speedQueue, overwriting if an old value is present.
            // Non-blocking call.
            xQueueOverwrite(speedQueue, &speed);

            // 2) Auto-lock on motion
            // Conditions for auto-locking:
            // - Car is in Drive or Reverse.
            // - Speed is greater than 10 km/h.
            // - Doors are not already locked.
            // - Driver's door is closed.
            // - `wasHigh` is false (to prevent re-locking immediately after speed drops and rises again).
            if ((isGearDrive() || isGearReverse()) && speed > 10 && !doorLocked && !isDriverDoorOpen() && !wasHigh) {
                setDoorLockLed(1);   // Turn on the door lock indicator LED.
                doorLocked = true;   // Update global door lock state.
                wasHigh = true;      // Set flag indicating speed has exceeded the threshold.
            }

            // Reset `wasHigh` if speed drops to 10 or below while in Drive/Reverse.
            // This allows auto-locking to occur again if speed increases above 10.
            if ((isGearDrive() || isGearReverse()) && speed <= 10 && wasHigh) {
                wasHigh = false;
            }

            // 3) Manual-lock lever (PA6 level)
            // Conditions for manual locking:
            // - Manual lock lever is activated.
            // - Doors are not already locked.
            // - Driver's door is closed (safety: don't lock if door is open).
            if (isManualLockOn() && !doorLocked && !isDriverDoorOpen()) {
                setDoorLockLed(1);
                doorLocked = true;
            }

            // 4) Manual-unlock lever (PA7 level)
            // Conditions for manual unlocking:
            // - Manual unlock lever is activated.
            // - Doors are currently locked.
            if (isManualUnlockOn() && doorLocked) {
                setDoorLockLed(0);   // Turn off door lock indicator LED.
                doorLocked = false;  // Update global door lock state.
            }

            // 5) Door-status -> buzzer & LCD
            // Conditions for door open warning:
            // - Driver's door is open.
            // - Doors are not locked (warning for unsecured door).
            if (isDriverDoorOpen() && !doorLocked) {
                setOnBuzzer(); // Activate buzzer.
                doorOpenedBuzzer = true; // Set flag indicating door open buzzer is active.

                // Attempt to take LCD mutex with a timeout of 50ms.
                if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(50))) {
                    LCD_SetCursor(0, 0);             // Position cursor on LCD.
                    LCD_Print("Door Opened      "); // Print message (padded with spaces to clear previous text).
                    xSemaphoreGive(lcdMutex);        // Release LCD mutex immediately.
                }
            } else {
                // If door is closed or locked, potentially stop the buzzer.
                doorOpenedBuzzer = false; // Clear door open buzzer flag.
                // Only turn off the buzzer if the distance buzzer isn't also trying to use it.
                if(!distanceBuzzer) {
                    setOffBuzzer();
                }
            }
        } else {
            // Ignition is off:
            // Ensure doors are unlocked and buzzer is off for safety/convenience.
            setOffBuzzer();      // Turn off buzzer.
            doorOpenedBuzzer = false;
            distanceBuzzer = false; // Also ensure distance buzzer flag is reset.
            setDoorLockLed(0);
            doorLocked = false;
            wasHigh = false; // Reset speed threshold flag.
        }

        // Task delay: run this task approximately every 50ms (20 Hz).
        // Reduced from 100ms for potentially better responsiveness.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Task for rear parking assistance using an ultrasonic sensor.
 *
 * This task performs the following functions when ignition is on and gear is in Reverse:
 * 1. Reads distance from the ultrasonic sensor.
 * 2. Sends the distance to `distanceQueue`.
 * 3. Controls a buzzer and an RGB LED based on the measured distance:
 * - < 30 cm: Fast buzzer, Red LED.
 * - < 100 cm: Medium buzzer, Yellow LED.
 * - >= 100 cm: Slow buzzer, Green LED.
 * 4. If distance cannot be measured, turns off buzzer and RGB LED.
 * 5. If not in Reverse or ignition is off, turns off buzzer (if not used by doorTask) and RGB LED.
 *
 * @param pvParameters Unused.
 */
void rearAssistTask(void* pvParameters) {
    uint32_t dist = 0; // Variable to store measured distance.

    while (1) {
        if (isIgnitionOn() && isGearReverse()) {
            // Read the distance from the ultrasonic sensor.
            dist = ultrasonic_get_distance();

            // Ensure the distance value is valid (ultrasonic_get_distance might return 0 or a max value on error/timeout).
            // Assuming >0 is a valid reading. The driver caps at 400cm.
            if (dist > 0 && dist <= 400) { // Check against a reasonable max, e.g., 400cm.
                // Send distance to distanceQueue, overwriting if an old value is present.
                xQueueOverwrite(distanceQueue, &dist);

                // Adjust buzzer frequency and LED color based on distance.
                // Buzzer control is conditional on doorOpenedBuzzer to avoid conflict.
                if (dist < 30) { // Closest range
                    if (!doorOpenedBuzzer){ // Only activate if door buzzer is not already on.
                        setBuzzerFrequency(750); // Fast beep (750ms half-period implies ~0.67Hz tone toggle)
                        distanceBuzzer = true;   // Set flag indicating distance buzzer is active.
                    }
                    setRGBColor('R'); // Red LED (Danger)
                } else if (dist < 100) { // Medium range
                     if (!doorOpenedBuzzer){
                        setBuzzerFrequency(1500); // Medium beep (1500ms half-period implies ~0.33Hz tone toggle)
                        distanceBuzzer = true;
                    }
                    setRGBColor('Y'); // Yellow LED (Warning)
                } else { // Farthest range (still within detection)
                    if (!doorOpenedBuzzer){
                        setBuzzerFrequency(3000); // Slow beep (3000ms half-period implies ~0.16Hz tone toggle)
                        distanceBuzzer = true;
                    }
                    setRGBColor('G'); // Green LED (Safe)
                }
            } else {
                // Handle the case where distance could not be measured or is out of expected range.
                // Turn off this task's buzzer contribution.
                if (!doorOpenedBuzzer) { // Only turn off if door buzzer isn't active
                    setOffBuzzer();
                }
                distanceBuzzer = false; // Clear distance buzzer flag.
                setRGBColor('0');       // Turn off RGB LEDs (assuming '0' or other char means off).
                // Optionally, send a specific value to distanceQueue to indicate error, e.g., 0 or max_val+1
                uint32_t error_dist = 0; // Or a specific error code like 999 if LCD handles it
                xQueueOverwrite(distanceQueue, &error_dist);
            }
        } else {
            // Not in reverse or ignition is off.
            // Turn off buzzer (if not used by doorTask) and LEDs.
            if (!doorOpenedBuzzer) {
                setOffBuzzer();
            }
            distanceBuzzer = false; // Clear distance buzzer flag.
            setRGBColor('0');       // Turn off RGB LEDs.
             // Clear distance queue or send a neutral value if LCD expects continuous updates
            uint32_t neutral_dist = 0; // Or a value that LCD interprets as "not reversing"
            xQueueOverwrite(distanceQueue, &neutral_dist);
        }

        // Task delay: run this task approximately every 20ms (50 Hz) for responsiveness during reverse.
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/**
 * @brief Task to update the LCD with various system statuses.
 *
 * This task displays:
 * - "Car ON" / "Car OFF" messages on ignition state changes.
 * - Current Gear (N, D, R).
 * - Door Lock Status (L/UL).
 * - Speed (km/h) when in Drive or Reverse.
 * - Distance (cm) when in Reverse.
 * - "Door Opened" message (handled by doorTask, but this task clears/updates around it).
 *
 * It uses a mutex (`lcdMutex`) to prevent concurrent access to the LCD.
 * It tries to minimize LCD writes by only updating changed information.
 *
 * @param pvParameters Unused.
 */
void lcdUpdateTask(void* pvParameters) {
    char buffer[17];               // Buffer for formatting strings for LCD (16 chars + null).
    int speed = 0, lastSpeed = -1; // Current and last known speed.
    uint32_t dist = 0, lastDist = -1; // Current and last known distance.
    static bool wasOn = false;     // Tracks if ignition was previously on.
    static char lastGear = 'X';    // Last known gear ('X' = uninitialized).
    static bool lastLock = false;  // Last known lock state.
    static bool doorWasOpen = false;// Tracks if the driver's door was previously open to manage LCD screen state.

    while (1) {
        if (isIgnitionOn()) {
            // If ignition just turned on, display "Car ON" message.
            if (!wasOn) {
                wasOn = true;
                if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(50))) { // Try to take mutex.
                    LCD_Clear();
                    LCD_SetCursor(0, 0);
                    LCD_Print("Car ON");
                    delay_ms(2000); // Display message for 2 seconds.
                    LCD_Clear();
                    xSemaphoreGive(lcdMutex); // Release mutex.
                }
                 // Reset last states to force an update of all fields
                lastGear = 'X';
                lastLock = !doorLocked; // Force update by choosing opposite of current
                lastSpeed = -1;
                lastDist = -1;
            }

            // Determine current gear, lock status, and door status.
            char gear = isGearDrive() ? 'D' : (isGearReverse() ? 'R' : 'N');
            bool lock = doorLocked; // Use the global doorLocked flag.
            bool doorOpen = isDriverDoorOpen();

            // Receive speed from speedQueue (non-blocking).
            xQueueReceive(speedQueue, &speed, 0);
            // Receive distance from distanceQueue if in reverse (non-blocking).
            if (isGearReverse()) {
                xQueueReceive(distanceQueue, &dist, 0);
            } else {
                dist = 0; // Clear distance if not in reverse for display purposes.
            }

            if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(50))) {
                // Handle LCD display when door is opened.
                // The "Door Opened" message is printed by doorTask.
                // This logic helps in restoring the screen after the door is closed.
                if (doorOpen && !doorWasOpen) {
                    doorWasOpen = true; // Mark that door has been opened.
                    // The actual "Door Opened" message is printed by doorTask.
                    // Here, we just note the state change.
                }

                // If door was open and is now closed, and doors are not locked,
                // reprint the standard info to overwrite "Door Opened" message.
                if (!doorOpen && doorWasOpen && !doorLocked) {
                    LCD_SetCursor(0, 0);
                    sprintf(buffer, "Gear:%c ", gear);
                    LCD_Print(buffer);
                    LCD_SetCursor(0, 7); // Position for lock status.
                    LCD_Print(lock ? "Doors:L  " : "Doors:UL "); // Padded for clearing.
                    lastGear = gear; // Update last known states.
                    lastLock = lock;
                    doorWasOpen = false; // Reset door open tracking.
                     // Force speed/distance update as well
                    lastSpeed = -1;
                    lastDist = -1;
                } else if (!doorOpen) { // If door is closed (and wasn't just closed overriding a message)
                                        // or if it is open but locked (so "Door Opened" message isn't shown)
                    // Update Gear and Lock status only if they changed.
                    if (gear != lastGear || lock != lastLock) {
                        LCD_SetCursor(0, 0);
                        sprintf(buffer, "Gear:%c ", gear);
                        LCD_Print(buffer);
                        LCD_SetCursor(0, 7);
                        LCD_Print(lock ? "Doors:L  " : "Doors:UL ");
                        lastGear = gear;
                        lastLock = lock;
                    }
                }
                // If door is open and unlocked, doorTask handles the "Door Opened" message on line 0.
                // So, we don't print gear/lock status to avoid conflict.

                // Update Speed and Distance on the second line.
                bool needUpdateLine2 = false;
								if (gear == 'R') { // In Reverse: display Speed and Distance.
										if (speed != lastSpeed || dist != lastDist || gear != lastGear) { // Update if speed, dist or gear changed (gear change implies context switch for line 2)
												sprintf(buffer, "S:%3dkm D:%3dcm ", speed, dist);
												lastSpeed = speed;
												lastDist = dist;
												needUpdateLine2 = true;
										}
								} else if (gear == 'D') { // In Drive: display Speed only.
										if (speed != lastSpeed || gear != lastGear) {
												sprintf(buffer, "S:%3dkm        ", speed); // Padded to clear distance.
												lastSpeed = speed;
												lastDist = 0; // Reset lastDist as it's not displayed.
												needUpdateLine2 = true;
										}
								} else if (gear == 'N') { // In Neutral: clear Speed and Distance.
										if (lastSpeed != -999 || lastDist != -999 || gear != lastGear) { // Use specific values to ensure clearing only once.
												sprintf(buffer, "                "); // Clear line.
												lastSpeed = -999; // Sentinel for "cleared".
												lastDist = -999;  // Sentinel for "cleared".
												needUpdateLine2 = true;
										}
								}


                if (needUpdateLine2) {
                    LCD_SetCursor(1, 0); // Position for second line.
                    LCD_Print(buffer);
                }
                xSemaphoreGive(lcdMutex); // Release mutex.
            }
        } else {
            // Ignition is off.
            // If ignition just turned off, display "Car OFF" message.
            if (wasOn) {
                wasOn = false;
                if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(50))) {
                    LCD_Clear();
                    LCD_SetCursor(0, 0);
                    LCD_Print("Car OFF");
                    delay_ms(2000); // Display message for 2 seconds.
                    LCD_Clear();
                    xSemaphoreGive(lcdMutex);
                }
                // Reset last states for next ignition ON cycle.
                lastGear = 'X';
                lastLock = false;
                lastSpeed = -1;
                lastDist = -1;
                doorWasOpen = false;
            }
        }
        // Task delay: update LCD approximately every 100ms (10 Hz).
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
