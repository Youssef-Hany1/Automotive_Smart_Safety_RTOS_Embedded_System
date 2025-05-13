/**
 * @file safety_tasks.h
 * @brief Header file for the vehicle safety system tasks.
 *
 * This file provides the public interface for creating and initializing the
 * FreeRTOS tasks that manage various safety features of a vehicle system.
 * These tasks include door locking logic, rear parking assistance, and
 * updating an LCD display with relevant status information.
 *
 * The primary function declared here, `createSafetyTasks()`, is responsible
 * for setting up these concurrent tasks and any necessary inter-task
 * communication mechanisms (like queues and semaphores) which are defined
 * within the corresponding .c file (e.g., safety_tasks.c or main.c).
 *
 * This module relies on FreeRTOS for task management and synchronization.
 * It also interacts with peripheral drivers defined in `drivers.h` and `lcd.h`.
 */

#ifndef SAFETY_TASKS_H
#define SAFETY_TASKS_H

// No specific includes are strictly necessary for this header file itself,
// as it only declares a function. However, the corresponding .c file
// will include FreeRTOS headers (task.h, queue.h, semphr.h),
// driver headers (drivers.h, lcd.h), and standard libraries.

//*****************************************************************************
//
// Public Function Prototypes
//
//*****************************************************************************

/**
 * @brief Creates and initializes the FreeRTOS tasks for the safety system.
 *
 * This function is responsible for:
 * 1. Creating synchronization primitives such as mutexes (e.g., for LCD access)
 * and queues (e.g., for passing speed and distance data between tasks).
 * 2. Creating the individual FreeRTOS tasks that implement the safety features:
 * - `doorTask`: Manages automatic and manual door locking, speed monitoring,
 * and driver door status warnings.
 * - `rearAssistTask`: Handles rear parking assistance using an ultrasonic sensor,
 * providing auditory and visual feedback.
 * - `lcdUpdateTask`: Updates an LCD display with system status information like
 * gear, speed, distance, and lock status.
 *
 * Each task is created with a specific stack size and priority, defined
 * within the implementation of this function. This function should be called
 * once during system initialization, typically after the hardware and
 * FreeRTOS scheduler have been started.
 */
void createSafetyTasks(void);

#endif // SAFETY_TASKS_H
