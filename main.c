/************************************************************************************************************************************************
*
*	File Name: main.c
*
*	Project: Automotive Smart Safety System (Intelligent Door Lock + Rear Park Assist) - Idea 2
*
*	Author: Team X
*
* Description: Main application file containing task definitions, global variables,
* RTOS object creation, task creation, and the main function to start
* the scheduler. Uses FreeRTOS on TM4C123GH6PM. Includes input debouncing.
*
*************************************************************************************************************************************************/

// --- Standard and Library Includes ---
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h> // For printf debugging if UART is configured

// --- FreeRTOS Includes ---
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// --- TivaWare Includes ---
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"   // Included via peripherals.h/Port_Config.h usually
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"    // For INT_GPIOx defines
#include "inc/hw_memmap.h"

// --- Project-Specific Includes ---
#include "Port_Config.h"    // Hardware pin definitions (MUST BE CONFIGURED BY USER)
#include "peripherals.h"    // Peripheral control function declarations
#include "tasks.h"          // Task function declarations and shared types

// --- Constants ---
#define DEBOUNCE_CHECKS 3 // Number of consecutive checks for stable input

// --- Global Variable Definitions ---
// Task Handles
TaskHandle_t xReadSpeedTaskHandle = NULL;
TaskHandle_t xReadGearTaskHandle = NULL;
TaskHandle_t xReadDoorStatusTaskHandle = NULL;
TaskHandle_t xReadManualControlsTaskHandle = NULL;
TaskHandle_t xReadIgnitionTaskHandle = NULL;
TaskHandle_t xDoorLockControlTaskHandle = NULL;
TaskHandle_t xReadUltrasonicTaskHandle = NULL;
TaskHandle_t xParkingAssistFeedbackTaskHandle = NULL;
TaskHandle_t xUpdateDisplayTaskHandle = NULL;

// Semaphores & Mutexes
SemaphoreHandle_t xLockStatusMutex = NULL; // Protects 'doorsLocked' & hardware access
SemaphoreHandle_t xDisplayMutex = NULL;    // Protects LCD access

// Queues
QueueHandle_t xSpeedQueue = NULL;          // Queue for int speed values (km/h)
QueueHandle_t xDistanceQueue = NULL;       // Queue for int distance values (cm)
QueueHandle_t xSystemStatusQueue = NULL;   // Queue for SystemStatusUpdate structs

// System State Variables (volatile if accessed by ISRs and tasks)
volatile int calculatedSpeed = 0;           // Latest calculated speed
volatile int calculatedDistance = 999;      // Latest calculated distance (999=inactive)
volatile GearState currentGear = PARK;      // Current gear state
volatile bool ignitionOn = false;           // Ignition status
volatile bool driverDoorOpen = false;       // Driver door status (true = open)
volatile bool doorsLocked = false;          // Current lock status (true = locked)
volatile bool manualLockRequest = false;    // Flag set by ISR/task for manual lock
volatile bool manualUnlockRequest = false;  // Flag set by ISR/task for manual unlock
volatile bool obstacleAlertActive = false;  // Parking assist buzzer/LED is active

// --- Main Function ---
int main()
{
    // Configure System Clock (Example: 80MHz from PLL)
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Initialize Peripherals using functions from peripherals.c
    Input_Switches_Init();
    Lock_Mechanism_Init();
    ADC_Init();
    Ultrasonic_Timer_Init();
    LCD_Init();
    Buzzer_Init();
    RGB_LED_Init();

    // --- Initialize RTOS Objects ---
    xLockStatusMutex = xSemaphoreCreateMutex();
    xDisplayMutex = xSemaphoreCreateMutex();
    xSpeedQueue = xQueueCreate(5, sizeof(int));
    xDistanceQueue = xQueueCreate(5, sizeof(int));
    xSystemStatusQueue = xQueueCreate(10, sizeof(SystemStatusUpdate));

    // Check if object creation succeeded (add error handling if necessary)
    if (xLockStatusMutex == NULL || xDisplayMutex == NULL || xSpeedQueue == NULL || xDistanceQueue == NULL || xSystemStatusQueue == NULL) {
        printf("ERROR: Failed to create RTOS objects!\n");
        while(1); // Halt on error
    }

    // --- Configure Interrupts (Example for Manual Controls on Port F) ---
    // If using polling tasks with debounce, ISRs might not be needed for those inputs,
    // but they are kept here as per the previous structure. Debounce will happen in the task.
    GPIOIntDisable(MANUAL_CTRL_PORT, MANUAL_LOCK_PIN | MANUAL_UNLOCK_PIN); // Disable during config
    GPIOIntClear(MANUAL_CTRL_PORT, MANUAL_LOCK_PIN | MANUAL_UNLOCK_PIN);   // Clear any pending interrupts
    GPIOIntTypeSet(MANUAL_CTRL_PORT, MANUAL_LOCK_PIN | MANUAL_UNLOCK_PIN, GPIO_FALLING_EDGE); // Trigger on press (assuming pull-ups)
    IntPrioritySet(MANUAL_CTRL_INT_BASE, configMAX_SYSCALL_INTERRUPT_PRIORITY + 1); // Set priority safe for FreeRTOS API calls from ISR
    GPIOIntRegister(MANUAL_CTRL_PORT, ISRHandlers); // Register the common ISR
    GPIOIntEnable(MANUAL_CTRL_PORT, MANUAL_LOCK_PIN | MANUAL_UNLOCK_PIN);   // Enable interrupts

    // Add similar interrupt configuration for Gear, Door, Ignition if using interrupts for them.

    // Enable processor interrupts.
    IntMasterEnable();

    // --- Create Tasks ---
    // Priorities: Higher number = higher priority
    xTaskCreate(vReadSpeedTask, "ReadSpeed", 128, NULL, 3, &xReadSpeedTaskHandle);
    xTaskCreate(vReadUltrasonicTask, "ReadUltra", 128, NULL, 3, &xReadUltrasonicTaskHandle);
    xTaskCreate(vReadGearTask, "ReadGear", 128, NULL, 2, &xReadGearTaskHandle);
    xTaskCreate(vReadDoorStatusTask, "ReadDoor", 128, NULL, 2, &xReadDoorStatusTaskHandle);
    xTaskCreate(vReadManualControlsTask, "ReadManual", 128, NULL, 2, &xReadManualControlsTaskHandle); // Reads buttons or checks ISR flags
    xTaskCreate(vReadIgnitionTask, "ReadIgnition", 128, NULL, 2, &xReadIgnitionTaskHandle);
    xTaskCreate(vDoorLockControlTask, "DoorLockCtrl", 192, NULL, 4, &xDoorLockControlTaskHandle); // Higher priority for core logic
    xTaskCreate(vParkingAssistFeedbackTask, "ParkAssistFB", 128, NULL, 3, &xParkingAssistFeedbackTaskHandle);
    xTaskCreate(vUpdateDisplayTask, "UpdateLCD", 200, NULL, 1, &xUpdateDisplayTaskHandle); // Lower priority for display

    printf("DEBUG: Tasks Created. Starting Scheduler...\n");

    // --- Start Scheduler ---
    vTaskStartScheduler();

    // --- Should never reach here ---
    while (1);
}


// --- Task Definitions ---

/* Task: Reads speed from potentiometer via ADC (No debounce needed for ADC) */
void vReadSpeedTask(void *pvParameters) {
    uint32_t rawValue;
    int speed;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // Run every 200ms

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait for the next cycle

        rawValue = Read_Potentiometer();
        speed = Calculate_Speed(rawValue);
        calculatedSpeed = speed; // Update global state

        // Send speed to queue for other tasks
        xQueueSend(xSpeedQueue, &speed, 0); // Don't block if queue is full
    }
}

/* Task: Reads gear selector switches with Debouncing */
void vReadGearTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // Poll more frequently for debounce (e.g., 25ms)
    GearState lastReading = GEAR_UNKNOWN; // Store the last raw reading
    GearState debouncedState = currentGear; // Start with the current global state
    uint8_t stableCount = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Read raw input states
        // IMPORTANT: Adapt active LOW/HIGH logic based on your hardware (pull-ups/downs)
        bool park_signal = !GPIOPinRead(GEAR_PORT, PARK_PIN);     // Example: Active LOW (pressed)
        bool reverse_signal = !GPIOPinRead(GEAR_PORT, REVERSE_PIN); // Example: Active LOW
        bool drive_signal = !GPIOPinRead(GEAR_PORT, DRIVE_PIN);   // Example: Active LOW

        // Determine current raw reading
        GearState currentReading = GEAR_UNKNOWN;
        if (park_signal) {
            currentReading = PARK;
        } else if (reverse_signal) {
            currentReading = REVERSE;
        } else if (drive_signal) {
            currentReading = DRIVE;
        } else {
            currentReading = PARK; // Default to PARK if nothing active
        }

        // Debounce logic
        if (currentReading == lastReading) {
            // Reading is the same as the last one
            stableCount++;
            if (stableCount >= DEBOUNCE_CHECKS) {
                // Signal has been stable long enough
                if (currentReading != debouncedState) {
                    debouncedState = currentReading;
                    // Update the global state only once it's debounced
                    currentGear = debouncedState;
                    // Optional: Log or signal gear change event here
                    printf("DEBUG: Gear changed to %d (debounced)\n", currentGear);
                }
                // Keep stableCount high to avoid re-triggering unless state changes
                stableCount = DEBOUNCE_CHECKS;
            }
        } else {
            // Reading changed, reset counter
            stableCount = 0;
        }

        // Update last reading for the next cycle
        lastReading = currentReading;
    }
}

/* Task: Reads driver door switch with Debouncing */
void vReadDoorStatusTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // Poll frequently for debounce
    bool lastReading = driverDoorOpen; // Store the last raw reading
    bool debouncedState = driverDoorOpen; // Start with global state
    uint8_t stableCount = 0;

    while(1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Read raw input state
        // Example: Assuming active LOW when door is OPEN
        bool currentReading = !GPIOPinRead(DOOR_PORT, DOOR_PIN);

        // Debounce logic
        if (currentReading == lastReading) {
            stableCount++;
            if (stableCount >= DEBOUNCE_CHECKS) {
                if (currentReading != debouncedState) {
                    debouncedState = currentReading;
                    // Update global state
                    driverDoorOpen = debouncedState;
                     printf("DEBUG: Door state changed to %s (debounced)\n", driverDoorOpen ? "Open" : "Closed");
                }
                stableCount = DEBOUNCE_CHECKS;
            }
        } else {
            stableCount = 0;
        }
        lastReading = currentReading;
    }
}

/* Task: Reads manual lock/unlock buttons with Debouncing */
void vReadManualControlsTask(void *pvParameters) {
    // This task handles debouncing for the manual buttons and sets flags.
    // The ISR might still trigger, but this task confirms the press.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // Poll frequently

    // State for Lock Button
    bool lock_lastReading = true; // Assume released (HIGH with pull-up)
    bool lock_debouncedState = true; // Assume released
    uint8_t lock_stableCount = 0;

    // State for Unlock Button
    bool unlock_lastReading = true; // Assume released (HIGH with pull-up)
    bool unlock_debouncedState = true; // Assume released
    uint8_t unlock_stableCount = 0;

    while(1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // --- Debounce Lock Button ---
        bool lock_currentReading = GPIOPinRead(MANUAL_CTRL_PORT, MANUAL_LOCK_PIN);
        if (lock_currentReading == lock_lastReading) {
            lock_stableCount++;
            if (lock_stableCount >= DEBOUNCE_CHECKS) {
                // Check for a falling edge on the *debounced* state
                if (lock_debouncedState && !lock_currentReading) {
                    manualLockRequest = true; // Set flag on confirmed press
                    printf("DEBUG: Manual Lock Request (debounced)\n");
                }
                lock_debouncedState = lock_currentReading; // Update stable state
                lock_stableCount = DEBOUNCE_CHECKS;
            }
        } else {
            lock_stableCount = 0; // Reset counter on change
        }
        lock_lastReading = lock_currentReading;

        // --- Debounce Unlock Button ---
        bool unlock_currentReading = GPIOPinRead(MANUAL_CTRL_PORT, MANUAL_UNLOCK_PIN);
         if (unlock_currentReading == unlock_lastReading) {
            unlock_stableCount++;
            if (unlock_stableCount >= DEBOUNCE_CHECKS) {
                 // Check for a falling edge on the *debounced* state
                if (unlock_debouncedState && !unlock_currentReading) {
                    manualUnlockRequest = true; // Set flag on confirmed press
                    printf("DEBUG: Manual Unlock Request (debounced)\n");
                }
                unlock_debouncedState = unlock_currentReading; // Update stable state
                unlock_stableCount = DEBOUNCE_CHECKS;
            }
        } else {
            unlock_stableCount = 0; // Reset counter on change
        }
        unlock_lastReading = unlock_currentReading;
    }
}

/* Task: Reads ignition switch with Debouncing */
void vReadIgnitionTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // Poll frequently for debounce
    bool lastReading = ignitionOn; // Store the last raw reading
    bool debouncedState = ignitionOn; // Start with global state
    uint8_t stableCount = 0;

    while(1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Read raw input state
        // Example: Assuming HIGH when ignition is ON
        bool currentReading = GPIOPinRead(IGNITION_PORT, IGNITION_PIN);

        // Debounce logic
        if (currentReading == lastReading) {
            stableCount++;
            if (stableCount >= DEBOUNCE_CHECKS) {
                 if (currentReading != debouncedState) {
                    debouncedState = currentReading;
                    // Update global state
                    ignitionOn = debouncedState;
                    printf("DEBUG: Ignition state changed to %s (debounced)\n", ignitionOn ? "ON" : "OFF");
                }
                stableCount = DEBOUNCE_CHECKS;
            }
        } else {
            stableCount = 0;
        }
        lastReading = currentReading;
    }
}

/* Task: Controls door lock mechanism based on system state */
void vDoorLockControlTask(void *pvParameters) {
    int currentSpeed = 0;
    bool needsLock = false;
    bool needsUnlock = false;
    SystemStatusUpdate statusUpdate;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Evaluate every 100ms

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Check for speed updates from queue (non-blocking)
        if (xQueueReceive(xSpeedQueue, &currentSpeed, 0) == pdPASS) {
             // Speed updated from queue
        } else {
            currentSpeed = calculatedSpeed; // Use latest global value if queue empty
        }

        // Determine desired lock state based on *debounced* global states
        needsLock = false;
        needsUnlock = false;

        // 1. Manual Override (Highest Priority - flags set by debounced task)
        if (manualLockRequest) {
            needsLock = true;
            manualLockRequest = false; // Consume the request
        } else if (manualUnlockRequest) {
            needsUnlock = true;
            manualUnlockRequest = false; // Consume the request
        } else {
            // 2. Ignition OFF unlocks doors (using debounced ignitionOn)
            if (!ignitionOn) {
                needsUnlock = true;
            }
            // 3. Automatic Locking (Ignition ON, speed > threshold, not in PARK - use debounced states)
            else if (currentSpeed > SPEED_THRESHOLD && currentGear != PARK && ignitionOn) {
                 needsLock = true;
            }
            // 4. Automatic Unlocking (Ignition ON, in PARK - use debounced states) - Optional rule
             else if (currentGear == PARK && ignitionOn) {
                 needsUnlock = true;
             }
        }

        // --- Action Execution with Mutex ---
        if (xSemaphoreTake(xLockStatusMutex, pdMS_TO_TICKS(50)) == pdPASS) { // Wait max 50ms for mutex
            if (needsLock && !doorsLocked) {
                Lock_Doors();
                doorsLocked = true;
                // Send status update to LCD task
                sprintf(statusUpdate.statusMessage, "Locked");
                statusUpdate.value = 0;
                xQueueSend(xSystemStatusQueue, &statusUpdate, 0);

            } else if (needsUnlock && doorsLocked) {
                Unlock_Doors();
                doorsLocked = false;
                // Send status update to LCD task
                sprintf(statusUpdate.statusMessage, "Unlocked");
                statusUpdate.value = 0;
                xQueueSend(xSystemStatusQueue, &statusUpdate, 0);
            }
            xSemaphoreGive(xLockStatusMutex);
        } else {
             printf("WARN: DoorLockControlTask couldn't get mutex!\n");
        }

        // --- Driver Door Open Alert (using debounced states) ---
        bool doorAlertCondition = ignitionOn && driverDoorOpen && (currentGear == DRIVE || currentGear == REVERSE);

        if (doorAlertCondition) {
            if (!obstacleAlertActive) {
                 Activate_Buzzer(1000);
            }
            sprintf(statusUpdate.statusMessage, "Door Open!");
            statusUpdate.value = 0;
            xQueueSend(xSystemStatusQueue, &statusUpdate, 0);
        } else {
            if (!obstacleAlertActive) {
                Deactivate_Buzzer();
            }
        }
    }
}


/* Task: Triggers and reads ultrasonic sensor (No debounce needed) */
void vReadUltrasonicTask(void *pvParameters) {
    uint32_t echoDuration;
    int distance;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequencyActive = pdMS_TO_TICKS(75);
    const TickType_t xFrequencyInactive = pdMS_TO_TICKS(250);

    while (1) {
         // Use debounced gear state
         if (currentGear == REVERSE) {
            vTaskDelayUntil(&xLastWakeTime, xFrequencyActive);
            Trigger_Ultrasonic();
            echoDuration = Read_Ultrasonic_Echo_Timer();
            distance = Calculate_Distance(echoDuration);
            calculatedDistance = distance;
            xQueueSend(xDistanceQueue, &distance, 0);
         } else {
             vTaskDelayUntil(&xLastWakeTime, xFrequencyInactive);
             if (calculatedDistance != 999) {
                distance = 999;
                calculatedDistance = distance;
                xQueueSend(xDistanceQueue, &distance, 0);
             }
         }
    }
}

/* Task: Controls buzzer and RGB LED for parking assist */
void vParkingAssistFeedbackTask(void *pvParameters) {
    int distance;
    uint32_t buzzerFreq = 0;
    bool currentAlertState = false;

    while (1) {
        if (xQueueReceive(xDistanceQueue, &distance, portMAX_DELAY) == pdPASS) {
            currentAlertState = false;
            // Use debounced gear state
            if (currentGear == REVERSE && distance < SAFE_ZONE_DISTANCE && distance >= 0) {
                 currentAlertState = true;
                 if (distance < CAUTION_ZONE_DISTANCE) { // Danger Zone
                    Set_RGB_LED(255, 0, 0);
                    buzzerFreq = 2000;
                 } else { // Caution Zone
                    Set_RGB_LED(255, 255, 0);
                    buzzerFreq = 1000;
                 }
                 Activate_Buzzer(buzzerFreq);
            } else { // Safe Zone or not in Reverse
                Set_RGB_LED(0, 255, 0);
                // Use debounced door/ignition/gear states for door alert check
                bool doorAlertCondition = ignitionOn && driverDoorOpen && (currentGear == DRIVE || currentGear == REVERSE);
                if (!doorAlertCondition) {
                    Deactivate_Buzzer();
                }
                 buzzerFreq = 0;
            }
            obstacleAlertActive = currentAlertState;
        }
    }
}


/* Task: Updates the LCD display */
void vUpdateDisplayTask(void *pvParameters) {
    int displaySpeed = 0;
    int displayDistance = 999;
    char lockStatusStr[9] = "Unlocked";
    SystemStatusUpdate statusUpdate;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(300);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Process Status Updates from Queue
        while (xQueueReceive(xSystemStatusQueue, &statusUpdate, 0) == pdPASS) {
            if (xSemaphoreTake(xDisplayMutex, pdMS_TO_TICKS(50)) == pdPASS) {
                if (strstr(statusUpdate.statusMessage, "Locked")) {
                     strcpy(lockStatusStr, "Locked");
                     LCD_Display_Status(lockStatusStr);
                } else if (strstr(statusUpdate.statusMessage, "Unlocked")) {
                     strcpy(lockStatusStr, "Unlocked");
                     LCD_Display_Status(lockStatusStr);
                } else if (strstr(statusUpdate.statusMessage, "Door Open!")) {
                     LCD_Display_Message(statusUpdate.statusMessage);
                } else {
                    LCD_Display_Message(statusUpdate.statusMessage);
                }
                xSemaphoreGive(xDisplayMutex);
            }
        }

        // Get latest sensor values (use globals updated by other tasks)
        displaySpeed = calculatedSpeed;
        displayDistance = calculatedDistance;

        // Update LCD with combined info
        if (xSemaphoreTake(xDisplayMutex, pdMS_TO_TICKS(50)) == pdPASS) {
            LCD_Display_Speed(displaySpeed);
            // Use debounced gear state
            if (currentGear == REVERSE) {
                LCD_Display_Distance(displayDistance);
            } else {
                LCD_GotoXY(8, 1); // Position for distance display
                LCD_Puts("        "); // Clear 8 chars
            }
            // Refresh status line
            LCD_Display_Status(lockStatusStr);
            xSemaphoreGive(xDisplayMutex);
        } else {
            printf("WARN: UpdateDisplayTask couldn't get mutex!\n");
        }
    }
}

// --- Idle Task Hook ---
void vApplicationIdleHook( void ) {
	// SysCtlSleep();
}

// --- Stack Overflow Hook ---
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ) {
    printf("FATAL: Stack overflow detected in task '%s'!\n", pcTaskName);
    while(1);
}

// --- Malloc Failed Hook ---
void vApplicationMallocFailedHook( void ) {
    printf("FATAL: Malloc failed!\n");
    while(1);
}


// --- Interrupt Service Routine Handler ---
// Handles GPIO interrupts. For debounced inputs read by tasks,
// this ISR might only set a flag indicating *potential* change,
// letting the task confirm and debounce. Or, ISR can be removed
// for inputs handled purely by polling tasks.
// Current implementation sets flags directly on edge.
void ISRHandlers(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ui32IntStatusPortF; // Add other ports if needed

    // --- Check Port F (Manual Controls Example) ---
    ui32IntStatusPortF = GPIOIntStatus(MANUAL_CTRL_PORT, true); // Masked status

    if (ui32IntStatusPortF & MANUAL_LOCK_PIN) {
        GPIOIntClear(MANUAL_CTRL_PORT, MANUAL_LOCK_PIN);
        // ISR sets the flag immediately. Debounce task confirms.
        // manualLockRequest = true; // This might cause issues if task also polls/debounces
        // Consider removing flag setting here if vReadManualControlsTask handles everything.
        // OR: Use a different mechanism like a semaphore just to wake the task.
    }

    if (ui32IntStatusPortF & MANUAL_UNLOCK_PIN) {
        GPIOIntClear(MANUAL_CTRL_PORT, MANUAL_UNLOCK_PIN);
        // ISR sets the flag immediately. Debounce task confirms.
        // manualUnlockRequest = true; // See comment above.
    }

    // --- Add checks for other interrupt sources if used ---

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
