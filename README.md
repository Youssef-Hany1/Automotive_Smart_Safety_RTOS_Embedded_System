# Automotive Smart Safety System

This project implements an Automotive Smart Safety System using the Tiva C Series TM4C123GH6PM microcontroller and FreeRTOS. It integrates two main safety features: an Intelligent Door Locking system based on vehicle speed and a Rear Parking Assistance System using an ultrasonic sensor.

## Overview

The system is designed to enhance vehicle safety by automating door locking at certain speeds and assisting the driver during reverse parking maneuvers. It utilizes FreeRTOS for real-time task management, ensuring reliable and concurrent operation of various system functionalities. The project simulates real-world automotive safety systems, demonstrating key embedded design principles such as modularity, task scheduling, real-time responsiveness, and sensor interfacing.

## Key Features

* **Intelligent Door Locking:**
    * Doors automatically lock when the vehicle's speed (simulated by a potentiometer) exceeds a predefined threshold (e.g., 10 km/h) and the car is in Drive or Reverse.
    * Doors automatically unlock when the ignition is turned off.
    * Manual lock/unlock pushbuttons allow driver override.
    * Audible buzzer and LCD message ("Door Opened") if the driver's door is opened while doors are unlocked and ignition is on.
* **Rear Parking Assistance:**
    * Uses an HC-SR04 ultrasonic sensor to measure the distance to nearby objects when in reverse gear.
    * Buzzer frequency increases as the object gets closer.
    * RGB LED provides visual proximity indication:
        * Green: Safe zone (> 100 cm)
        * Yellow: Caution zone (30-100 cm)
        * Red: Danger zone (< 30 cm)
* **System Status Display:**
    * LCD displays current system status, including "Doors Locked", "Doors Unlocked", speed, gear, and ultrasonic distance.
    * "Car ON" / "Car OFF" messages on ignition state changes.
* **Real-Time Operation:**
    * Managed by FreeRTOS, with tasks for door control, rear assist, and LCD updates.
    * Uses queues for inter-task communication (speed and distance data) and mutexes for shared resource protection (LCD).

## Hardware Components

* **Microcontroller:** TM4C123GH6PM (Tiva C Series)
* **Sensors:**
    * Potentiometer (for speed simulation)
    * HC-SR04 Ultrasonic Sensor
* **User Interface:**
    * Pushbutton or Limit switch module for vehicle door simulation
    * Push buttons for manual lock/unlock
    * Switches to mimic gears (Neutral, Drive, Reverse)
    * Ignition Switch (simulated toggle input)
    * RGB LED
    * LCD display
    * Buzzer for audible alerts
* **Drivers:**
    * Custom peripheral drivers (`drivers.h`)
    * LCD driver (`lcd.h`)
    * TivaWare DriverLib

## Software Implementation

The firmware is built upon FreeRTOS, managing three primary tasks:

1.  **`doorTask`**:
    * Reads current speed via ADC and sends it to `speedQueue`.
    * Manages automatic door locking based on speed, gear, and driver door status.
    * Handles manual lock/unlock requests.
    * Activates a buzzer and displays "Door Opened" on the LCD if the driver's door is opened while unlocked with ignition on.
    * Ensures doors are unlocked and buzzer is off if ignition is off.
    * *Periodicity*: Approximately every 50ms (20 Hz).
2.  **`rearAssistTask`**:
    * Reads distance from the ultrasonic sensor when in reverse.
    * Sends distance data to `distanceQueue`.
    * Controls buzzer frequency and RGB LED color based on measured distance.
    * Turns off its buzzer contribution and RGB LED if distance cannot be measured or if not in reverse/ignition off.
    * *Periodicity*: Approximately every 20ms (50 Hz).
3.  **`lcdUpdateTask`**:
    * Displays "Car ON" / "Car OFF" messages on ignition state changes.
    * Shows current Gear (N, D, R), Door Lock Status (L/UL), Speed (km/h), and Distance (cm) on the LCD.
    * Manages the LCD display updates to avoid conflicts, especially when the "Door Opened" message is active.
    * *Periodicity*: Approximately every 100ms (10 Hz).

**Synchronization Primitives:**

* `speedQueue`: A FreeRTOS queue (size 1, `int`) to send speed data from `doorTask` to `lcdUpdateTask`. Overwrites old data.
* `distanceQueue`: A FreeRTOS queue (size 1, `uint32_t`) to send ultrasonic distance data from `rearAssistTask` to `lcdUpdateTask`. Overwrites old data.
* `lcdMutex`: A FreeRTOS mutex to protect shared access to the LCD among tasks.

**Global State Flags:**

* `doorLocked`: `bool`, true if doors are locked.
* `doorOpenedBuzzer`: `bool`, true if the door open buzzer is active (controlled by `doorTask`).
* `distanceBuzzer`: `bool`, true if the rear assist distance buzzer is active (controlled by `rearAssistTask`).

## How to Use/Setup

1.  **Hardware Connections:**
    * Connect all sensors (potentiometer, ultrasonic sensor).
    * Connect actuators (simulated door lock mechanism/LED, buzzer, RGB LED).
    * Connect UI elements (buttons for manual lock/unlock, switches for gear and ignition, driver door sensor, LCD).
2.  **Build and Flash:**
    * Compile the project using an ARM GCC toolchain (e.g., as part of Code Composer Studio or a Makefile-based setup).
    * Ensure FreeRTOS source files and TivaWare libraries are correctly linked.
    * Flash the compiled firmware onto the Tiva C TM4C123GH6PM microcontroller.
3.  **Operation:**
    * **Ignition:** Use the designated switch to simulate turning the ignition ON or OFF.
    * **Gears:** Use switches to select Neutral (N), Drive (D), or Reverse (R).
    * **Speed:** Adjust the potentiometer to simulate vehicle speed.
    * **Manual Lock/Unlock:** Use the dedicated pushbuttons.
    * **Driver Door:** Use the switch/sensor to simulate opening and closing the driver's door.
    * **Observation:**
        * The LCD will display system status (Ignition, Gear, Lock Status, Speed, Distance).
        * The buzzer and RGB LED will provide feedback for the rear parking assist when in Reverse.
        * The buzzer will sound if the driver's door is open while the ignition is on and doors are unlocked.

## Future Improvements

* Integration of CAN bus for communication with other potential vehicle ECUs.
* More sophisticated speed detection mechanisms (e.g., using timer input capture with an encoder).
* Enhanced power management features for low-power modes.
* Error handling and reporting for sensor malfunctions.
* Calibration mode for sensors.
