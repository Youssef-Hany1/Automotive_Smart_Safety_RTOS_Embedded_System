//*****************************************************************************
//
// Includes and Global Definitions
//
//*****************************************************************************
#include "drivers.h"        // Custom driver definitions (ensure this file exists and is relevant)
#include "tm4c123gh6pm.h"   // MCU-specific header for TM4C123GH6PM
#include "TM4C123.h"        // Alternative MCU-specific header (often a more generic CMSIS style)
#include <stdint.h>         // Standard integer types (e.g., uint32_t)
#include <stdbool.h>        // Standard boolean type (true, false)

// TivaWare DriverLib Includes
#include "Drivers/driverlib/sysctl.h"   // System Control library (clocking, peripherals)
#include "Drivers/driverlib/gpio.h"     // General Purpose Input/Output library
#include "Drivers/driverlib/adc.h"      // Analog-to-Digital Converter library
#include "Drivers/driverlib/pin_map.h"  // GPIO pin mapping (e.g., GPIO_PB4_T1CCP0)
#include "Drivers/driverlib/timer.h"    // Timer library
#include "Drivers/driverlib/interrupt.h"// Interrupt controller library

// TivaWare Hardware Abstraction Layer (HWREG) Includes
#include "Drivers/inc/hw_gpio.h"    // Hardware GPIO registers
#include "Drivers/inc/hw_ints.h"    // Hardware interrupt numbers
#include "Drivers/inc/hw_memmap.h"  // Hardware memory map

// Timer 0A (used for short delays, e.g., ultrasonic trigger pulse)
#define TIMER0A_PERIPH          SYSCTL_PERIPH_TIMER0 // Peripheral ID for Timer 0
#define TIMER0A_BASE            TIMER0_BASE          // Base address for Timer 0
#define TIMER0A_TIMER           TIMER_A              // Use Timer A of Timer 0

// Timer 1A (used for ultrasonic echo capture)
#define TIMER1A_PERIPH          SYSCTL_PERIPH_TIMER1 // Peripheral ID for Timer 1
#define TIMER1A_BASE            TIMER1_BASE          // Base address for Timer 1
#define TIMER1A_TIMER           TIMER_A              // Use Timer A of Timer 1
#define TIMER1A_INT             INT_TIMER1A          // Interrupt vector for Timer 1A
#define TIMER1A_EVENT           TIMER_CAPA_EVENT     // Capture event for Timer 1A

// GPIO for Timer 1A Input Capture (Ultrasonic Echo Pin)
#define TIMER1A_GPIO_PERIPH     SYSCTL_PERIPH_GPIOB // Peripheral ID for GPIO Port B
#define TIMER1A_GPIO_PORT       GPIO_PORTB_BASE     // Base address for GPIO Port B
#define TIMER1A_GPIO_PIN        GPIO_PIN_4          // GPIO Pin B4 for Timer 1A capture
#define TIMER1A_GPIO_PIN_CONFIG GPIO_PB4_T1CCP0     // Pin mux configuration for PB4 as T1CCP0

// Ultrasonic Sensor Definitions
#define ULTRASONIC_TICKS_TO_US (SysCtlClockGet() / 1000000) // Conversion factor: clock ticks per microsecond
#define ULTRASONIC_PERIPH_TRIGGER SYSCTL_PERIPH_GPIOE      // Peripheral ID for GPIO Port E (Trigger Pin)
#define ULTRASONIC_PORT_TRIGGER   GPIO_PORTE_BASE          // Base address for GPIO Port E
#define ULTRASONIC_TRIGGER_PIN    GPIO_PIN_4               // GPIO Pin E4 for Ultrasonic Trigger
#define ULTRASONIC_TICK_TIME      (1.0f / SysCtlClockGet()) // Time duration of a single clock tick (used for float calculations if needed)

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

// Variables for Ultrasonic Sensor measurement
volatile uint32_t risingEdge = 0;  // Timestamp of the rising edge of the echo pulse
volatile uint32_t fallingEdge = 0; // Timestamp of the falling edge of the echo pulse
volatile bool captured = false;    // Flag to indicate if an echo pulse has been fully captured

// Global volatile flags for system state, modified by ISRs
volatile int ignitionState = 0;     // 0 for off, 1 for on
volatile int driverDoorState = 0;   // 0 for closed, 1 for open

// Latch for gear switch state
volatile Gear_t gearState = GEAR_NEUTRAL; // Current gear state

volatile bool manualLockState   = false; // True if manual lock lever is activated
volatile bool manualUnlockState = false; // True if manual unlock lever is activated

//*****************************************************************************
//
// Delay Functions
//
//*****************************************************************************

/**
 * @brief Provides a short debounce delay.
 *
 * This function uses SysCtlDelay to create a delay of approximately 1 ms,
 * assuming the system clock is relatively stable. The division factor (3000)
 * is empirical and might need adjustment based on the exact clock frequency
 * and desired delay.
 */
void debounceDelay() {
    SysCtlDelay(SysCtlClockGet() / 3000); // Approx 1ms delay (SysCtlDelay takes 3 clock cycles per loop)
}

/**
 * @brief Provides a delay in milliseconds.
 *
 * This function creates a blocking delay for the specified number of milliseconds.
 * The inner loop count (1334) is calibrated for a ~1ms delay at a 16MHz system clock.
 * This will be inaccurate if the system clock is different.
 * For more accurate and non-blocking delays, a hardware timer is preferred.
 *
 * @param ms The number of milliseconds to delay.
 */
void delay_ms(uint32_t ms)
{
    volatile unsigned int i, j;
    for (i = 0; i < ms; i++)
    {
        for (j = 0; j < 1334; j++)  // Calibrated for ~1ms delay at 16MHz.
        {
            // This loop consumes time.
        }
    }
}

//*****************************************************************************
//
// Switch Input Handling (GPIO Port A)
//
//*****************************************************************************

/**
 * @brief Interrupt Service Routine for GPIO Port A.
 *
 * This handler is triggered by edge changes on pins PA2-PA7.
 * It reads the status of the pins to update global state variables for
 * manual lock/unlock, ignition, driver door, and gear selection.
 * A debounce delay is applied after clearing the interrupt.
 */
void GPIOA_Handler(void) {
    uint32_t status = GPIOIntStatus(GPIO_PORTA_BASE, true); // Get interrupt status
    GPIOIntClear(GPIO_PORTA_BASE, status);                 // Clear the asserted interrupts

    debounceDelay(); // Apply a short delay to mitigate switch bounce

    // PA6 = lock lever: Reads low when switch pressed (connected to ground)
    if (status & GPIO_PIN_6) {
        // Update manualLockState based on the current pin level.
        // Assumes active low: pressed means state is true.
        manualLockState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0);
    }

    // PA7 = unlock lever: Reads low when switch pressed
    if (status & GPIO_PIN_7) {
        // Update manualUnlockState based on the current pin level.
        // Assumes active low.
        manualUnlockState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) == 0);
    }

    // PA4 = IGNITION lever: Reads low when switch pressed
    if (status & GPIO_PIN_4) {
        // Update ignitionState. Assumes active low.
        ignitionState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == 0);
    }

    // PA5 = DRIVER-DOOR lever: Reads low when switch pressed
    if (status & GPIO_PIN_5) {
        // Update driverDoorState. Assumes active low.
        driverDoorState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) == 0);
    }

    // Latch gear state on each button press (assuming momentary switches for gears)
    // --- DRIVE lever (PA2) ---
    if (status & GPIO_PIN_2) {
        // Reads 0 (low) when switch closed to ground
        if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == 0) { // Switch pressed
            gearState = GEAR_DRIVE;      // Set gear to DRIVE
        } else { // Switch released (or if it's a two-position switch returning to open)
            // This logic implies that releasing a gear switch might return to NEUTRAL.
            // If using toggle switches, this 'else' might need different logic.
            gearState = GEAR_NEUTRAL;    // Return to NEUTRAL
        }
    }

    // --- REVERSE lever (PA3) ---
    if (status & GPIO_PIN_3) {
        if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) == 0) { // Switch pressed
            gearState = GEAR_REVERSE;    // Set gear to REVERSE
        } else { // Switch released
            gearState = GEAR_NEUTRAL;    // Return to NEUTRAL
        }
    }
}

/**
 * @brief Initializes GPIO pins on Port A for switch inputs.
 *
 * Configures PA2-PA7 as inputs with internal pull-up resistors.
 * Sets up interrupts on these pins to trigger on both rising and falling edges.
 * This allows detection of switch presses and releases.
 */
void initSwitches() {
    // Enable the clock for GPIO Port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)); // Wait for the peripheral to be ready

    // Configure PA2-PA7 as input pins
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    // Enable internal pull-up resistors for PA2-PA7 and set 2mA drive strength
    // Pull-ups ensure a defined state (HIGH) when switches are open (assuming switches connect to ground when closed)
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Configure interrupts for PA2-PA7 (mask 0xFC = pins 2,3,4,5,6,7)
    GPIOIntDisable(GPIO_PORTA_BASE, 0xFC);          // Disable interrupts during configuration
    GPIOIntClear(GPIO_PORTA_BASE, 0xFC);            // Clear any pending interrupts
    GPIOIntRegister(GPIO_PORTA_BASE, GPIOA_Handler); // Register the interrupt handler for Port A
    GPIOIntTypeSet(GPIO_PORTA_BASE, 0xFC, GPIO_BOTH_EDGES); // Set interrupt type to trigger on both edges
    GPIOIntEnable(GPIO_PORTA_BASE, 0xFC);           // Enable interrupts for the specified pins
}

//*****************************************************************************
//
// Analog-to-Digital Converter (ADC) for Speed Sensor
//
//*****************************************************************************

/**
 * @brief Initializes the ADC module for reading an analog sensor.
 *
 * Configures ADC0, Sequence 3, to read from Channel 0 (AIN0), which is on PE3.
 * The ADC conversion is triggered by the processor.
 * An interrupt is enabled at the end of the sequence.
 */
void initADC() {
    // Enable the peripherals for ADC0 and GPIO Port E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Wait for the peripherals to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    // Configure PE3 as an ADC input pin (AIN0)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // Configure ADC0 Sequence 3
    // Triggered by the processor, highest priority (0)
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // Configure step 0 of sequence 3:
    // - Read from Channel 0 (ADC_CTL_CH0)
    // - Enable interrupt flag generation (ADC_CTL_IE)
    // - Mark this as the last step in the sequence (ADC_CTL_END)
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    // Enable ADC0 Sequence 3
    ADCSequenceEnable(ADC0_BASE, 3);
    // Clear any existing interrupt flags for this sequence
    ADCIntClear(ADC0_BASE, 3);
}

/**
 * @brief Reads the speed from the ADC and scales it.
 *
 * Triggers an ADC conversion on Sequence 3, waits for completion,
 * retrieves the raw digital result (0-4095), and scales it to
 * a speed value (0-120 km/h).
 *
 * @return int The calculated speed in km/h.
 */
int readSpeedADC() {
    uint32_t result; // Variable to store the raw ADC result

    // Trigger the ADC conversion on Sequence 3
    ADCProcessorTrigger(ADC0_BASE, 3);

    // Wait for the conversion to complete by checking the interrupt status flag
    // (This is a blocking wait)
    while (!ADCIntStatus(ADC0_BASE, 3, false));

    // Clear the ADC interrupt flag for Sequence 3
    ADCIntClear(ADC0_BASE, 3);

    // Retrieve the conversion result from ADC Sequence 3
    ADCSequenceDataGet(ADC0_BASE, 3, &result);

    // Convert the ADC result (0-4095) to speed (0-120 km/h)
    // Speed = (ADC_Result / ADC_Max_Value) * Max_Speed
    int speed = (result * 120) / 4095;
    return speed;
}

//*****************************************************************************
//
// Timer0 for Microsecond Delays
//
//*****************************************************************************

/**
 * @brief Initializes Timer0A as a one-shot timer.
 *
 * This timer is used for generating short, precise delays, such as the
 * trigger pulse for the ultrasonic sensor.
 */
void initTimer0(void) {
    // Enable the clock for Timer0 peripheral
    SysCtlPeripheralEnable(TIMER0A_PERIPH);
    while (!SysCtlPeripheralReady(TIMER0A_PERIPH)); // Wait for Timer0 to be ready

    // Disable Timer0A before configuration
    TimerDisable(TIMER0A_BASE, TIMER0A_TIMER);

    // Configure Timer0A as a 32-bit one-shot timer (split pair mode, using Timer A)
    TimerConfigure(TIMER0A_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT);
}

/**
 * @brief Creates a delay using Timer0A.
 *
 * Loads Timer0A with a value corresponding to the desired delay in microseconds
 * and waits for it to time out. This is a blocking delay.
 *
 * @param time_us The delay time in microseconds.
 */
void timer0_delay(uint32_t time_us) {
    // Load the timer with the delay value.
    // SysCtlClockGet() gives clock frequency in Hz (ticks/sec).
    // To get ticks for 'time_us' microseconds: (SysCtlClockGet() / 1,000,000) * time_us
    // The TivaWare timers typically count down from the load value.
    // A load value of N gives N+1 clock cycles. So, subtract 1 if precision is critical.
    // Original code had `time * 16 - 1`. Assuming 'time' was meant to be 'time_us'
    // and the system clock was 16MHz. This is now generalized.
    TimerLoadSet(TIMER0A_BASE, TIMER0A_TIMER, (SysCtlClockGet() / 1000000) * time_us -1);

    // Clear any pending timeout interrupt flags for Timer0A
    TimerIntClear(TIMER0A_BASE, TIMER_TIMA_TIMEOUT);

    // Enable Timer0A
    TimerEnable(TIMER0A_BASE, TIMER0A_TIMER);

    // Wait for Timer0A to time out (blocking wait)
    while ((TimerIntStatus(TIMER0A_BASE, false) & TIMER_TIMA_TIMEOUT) == 0);
}

//*****************************************************************************
//
// Timer1 for Ultrasonic Sensor Input Capture
//
//*****************************************************************************

/**
 * @brief Initializes Timer1A for input capture mode.
 *
 * Configures Timer1A to capture the time of both rising and falling edges
 * on pin PB4 (T1CCP0). This is used to measure the pulse width of the
 * ultrasonic sensor's echo signal. An interrupt is generated on capture events.
 *
 * @param handler Pointer to the interrupt handler function for Timer1A.
 */
void initTimer1(void (*handler)(void)) {
    // Enable the clock for Timer1 peripheral
    SysCtlPeripheralEnable(TIMER1A_PERIPH);
    while (!SysCtlPeripheralReady(TIMER1A_PERIPH)); // Wait for Timer1A to be ready

    // Enable the clock for GPIO Port B (used for Timer1A capture pin)
    SysCtlPeripheralEnable(TIMER1A_GPIO_PERIPH);
    while (!SysCtlPeripheralReady(TIMER1A_GPIO_PERIPH)); // Wait for GPIOB to be ready

    // Configure PB4 as a timer input pin for Timer1A
    GPIOPinTypeTimer(TIMER1A_GPIO_PORT, TIMER1A_GPIO_PIN);
    // Set the alternate function for PB4 to be T1CCP0 (Timer1A Capture Compare Pin 0)
    GPIOPinConfigure(TIMER1A_GPIO_PIN_CONFIG);

    // Disable Timer1A before configuration
    TimerDisable(TIMER1A_BASE, TIMER1A_TIMER);

    // Configure Timer1A:
    // - Split pair mode (using Timer A independently)
    // - Input edge-time capture mode (TIMER_CFG_A_CAP_TIME_UP for up-counting)
    TimerConfigure(TIMER1A_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);

    // Load Timer1A with the maximum possible value (0xFFFF for 16-bit mode if using TIMER_CFG_A_CAP_TIME,
    // or 0xFFFFFFFF for 24-bit part of 32-bit timer if full width is used).
    // For 32-bit split timers, the "A" timer is typically 16-bit and "B" is 16-bit, or combined as 32-bit.
    // TIMER_CFG_A_CAP_TIME_UP implies a 24-bit timer on TM4C123 (16-bit main timer + 8-bit prescaler).
    // Let's assume it's intended to use the full available range. The example uses 0xFFFF,
    // but a 24-bit timer can go up to 0xFFFFFF. If it's a 16-bit timer part of a split pair, 0xFFFF is correct.
    // Given TIMER_CFG_A_CAP_TIME_UP, it's a 24-bit timer.
    TimerLoadSet(TIMER1A_BASE, TIMER1A_TIMER, 0xFFFFFF); // Max value for a 24-bit timer

    // Configure Timer1A to capture events on both rising and falling edges
    TimerControlEvent(TIMER1A_BASE, TIMER1A_TIMER, TIMER_EVENT_BOTH_EDGES);

    // Register the interrupt handler for Timer1A
    IntRegister(TIMER1A_INT, handler);
    // Enable the interrupt for Timer1A in the interrupt controller (NVIC)
    IntEnable(TIMER1A_INT);

    // Clear any pending capture event interrupts for Timer1A
    TimerIntClear(TIMER1A_BASE, TIMER1A_EVENT);
    // Enable the capture event interrupt for Timer1A at the timer level
    TimerIntEnable(TIMER1A_BASE, TIMER1A_EVENT);
}

/**
 * @brief Enables Timer1A.
 *
 * Starts Timer1A, allowing it to count and capture edge events.
 */
void timer1_enable(void) {
    TimerEnable(TIMER1A_BASE, TIMER1A_TIMER);
}

/**
 * @brief Disables Timer1A.
 *
 * Stops Timer1A from counting and capturing events.
 */
void timer1_disable(void) {
    TimerDisable(TIMER1A_BASE, TIMER1A_TIMER);
}

/**
 * @brief Clears the Timer1A capture event interrupt flag.
 */
void timer1_clear(void) {
    TimerIntClear(TIMER1A_BASE, TIMER1A_EVENT);
}

//*****************************************************************************
//
// Ultrasonic Distance Sensor Control
//
//*****************************************************************************

/**
 * @brief Initializes the ultrasonic sensor system.
 *
 * Sets up the GPIO pin for the trigger pulse as an output.
 * Initializes Timer0 for generating the trigger pulse delay.
 * Initializes Timer1 for capturing the echo pulse, with `ultrasonic_edge_processor`
 * as the interrupt handler.
 */
void initUltrasonic(void) {
    // Enable clock for GPIO Port E (for the trigger pin)
    SysCtlPeripheralEnable(ULTRASONIC_PERIPH_TRIGGER);
    while (!SysCtlPeripheralReady(ULTRASONIC_PERIPH_TRIGGER));

    // Configure the ultrasonic trigger pin (PE4) as a GPIO output
    GPIOPinTypeGPIOOutput(ULTRASONIC_PORT_TRIGGER, ULTRASONIC_TRIGGER_PIN);

    // Initialize Timer0 for creating the trigger pulse width
    initTimer0();
    // Initialize Timer1 for capturing the echo pulse edges, pass the ISR
    initTimer1(ultrasonic_edge_processor);
}

/**
 * @brief Sends a trigger pulse to the ultrasonic sensor.
 *
 * Generates a 10-microsecond high pulse on the trigger pin (PE4)
 * using Timer0 for the delay.
 */
void ultrasonic_trigger(void) {
    // Set the trigger pin high
    GPIOPinWrite(ULTRASONIC_PORT_TRIGGER, ULTRASONIC_TRIGGER_PIN, ULTRASONIC_TRIGGER_PIN);
    // Wait for 10 microseconds
    timer0_delay(10);
    // Set the trigger pin low
    GPIOPinWrite(ULTRASONIC_PORT_TRIGGER, ULTRASONIC_TRIGGER_PIN, 0);
}

/**
 * @brief Measures distance using the ultrasonic sensor.
 *
 * Enables Timer1 for echo capture, sends a trigger pulse, and waits for the
 * `captured` flag to be set by the Timer1 ISR (`ultrasonic_edge_processor`).
 * Calculates the distance in centimeters based on the time difference between
 * the rising and falling edges of the echo pulse.
 * The distance is capped at 400 cm.
 *
 * @return uint32_t The measured distance in centimeters. Returns 400 if out of range.
 */
uint32_t ultrasonic_get_distance(void) {
    captured = false; // Reset captured flag before measurement

    timer1_enable();  // Enable Timer1 to start capturing echo
    ultrasonic_trigger(); // Send the trigger pulse

    // Wait for the echo to be captured (the 'captured' flag is set in the ISR)
    // A timeout mechanism might be useful here to prevent an infinite loop if no echo is received.
    while (!captured);

    timer1_disable(); // Disable Timer1 after measurement is complete
    timer1_clear();   // Clear any pending Timer1 interrupt flags

    float timeDiff_ticks; // Time difference in timer ticks

    // Calculate the time difference, handling timer overflow
    if (fallingEdge >= risingEdge) {
        timeDiff_ticks = (float)(fallingEdge - risingEdge);
    } else {
        // Timer overflow occurred between rising and falling edge
        // Max timer value for a 24-bit timer is 0xFFFFFF
        timeDiff_ticks = (float)((0xFFFFFF - risingEdge) + fallingEdge);
    }

    // Reset the captured flag for the next measurement
    captured = false;

    // Calculate distance in cm:
    // Distance = (Time * Speed_of_Sound) / 2
    // Speed of sound ~34300 cm/s
    // Time_seconds = timeDiff_ticks / SysCtlClockGet()
    // Distance_cm = (timeDiff_ticks / SysCtlClockGet()) * 34300 / 2
    // Distance_cm = timeDiff_ticks * (34300 / (2 * SysCtlClockGet()))
    // Distance_cm = timeDiff_ticks * (17150 / SysCtlClockGet())
    //
    // timeDiff / 932.9446064
    // This implies SysCtlClockGet() / 17150 = 932.944...
    // So, SysCtlClockGet() = 932.944 * 17150 = ~16,000,000 (16 MHz)
    // We will use the formula directly for clarity, assuming SysCtlClockGet() is available and correct.
    float clock_freq = (float)SysCtlClockGet();
    float distance_cm = (timeDiff_ticks * 17150.0f) / clock_freq;

    // Clamp distance to a maximum of 400 cm
    if (distance_cm > 400.0f) {
        distance_cm = 400.0f;
    }
    if (distance_cm < 0) { // Should not happen with unsigned edges, but good practice
        distance_cm = 0;
    }

    return (uint32_t)distance_cm;
}

/**
 * @brief Interrupt Service Routine for Timer1A (Ultrasonic Echo Capture).
 *
 * This function is called when Timer1A captures an edge on the echo pin (PB4).
 * It reads the timer value at the edge.
 * If it's a rising edge, it stores the time in `risingEdge`.
 * If it's a falling edge, it stores the time in `fallingEdge` and sets the
 * `captured` flag to true, indicating a complete echo pulse has been measured.
 */
void ultrasonic_edge_processor(void) {
    // Clear the Timer1A capture event interrupt flag
    TimerIntClear(TIMER1A_BASE, TIMER1A_EVENT);

    // Check if the current state of the echo pin is HIGH (indicating a rising edge just occurred)
    if (GPIOPinRead(TIMER1A_GPIO_PORT, TIMER1A_GPIO_PIN) == TIMER1A_GPIO_PIN) { // Rising edge
        risingEdge = TimerValueGet(TIMER1A_BASE, TIMER1A_TIMER); // Capture the timer value
    } else { // Falling edge (pin is now LOW)
        fallingEdge = TimerValueGet(TIMER1A_BASE, TIMER1A_TIMER); // Capture the timer value
        captured = true; // Set the flag to indicate that the measurement is done
    }
}

//*****************************************************************************
//
// RGB LED Control (GPIO Port F)
//
//*****************************************************************************

/**
 * @brief Initializes GPIO pins on Port F for RGB LED control.
 *
 * Configures PF1 (Red) and PF2 (Green) as outputs.
 * Current code only uses PF1 and PF2.
 */
void initRGB() {
    // Enable the clock for GPIO Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Wait for the peripheral to be ready

    // Configure PF1 (Red LED) and PF2 (Green LED) as GPIO outputs.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2);

    // Initially turn off the LEDs (set pins low)
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0);
}

/**
 * @brief Sets the color of an RGB LED connected to PF1 and PF2.
 *
 * 'R' sets PF1 (Red) ON.
 * 'G' sets PF2 (Green) ON.
 * 'Y' sets both PF1 and PF2 ON (Yellow if PF1=R, PF2=G).
 * Any other character turns both LEDs OFF.
 *
 * @param color Character representing the desired color ('R', 'G', 'Y').
 */
void setRGBColor(char color) {
    switch (color) {
        case 'R': // Red
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_1);
            break;
        case 'G': // Green
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_2);
            break;
        case 'Y': // Yellow (Red + Green)
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_1 | GPIO_PIN_2);
            break;
        default:  // Off
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0);
            break;
    }
}

//*****************************************************************************
//
// Door Lock LED Control (GPIO Port F)
//
//*****************************************************************************

/**
 * @brief Initializes GPIO Pin PF4 for Door Lock LED indication.
 *
 * Configures PF4 as a GPIO output.
 */
void initDoorLockLed(void) {
    // Enable the clock for GPIO Port F (may already be enabled by initRGB)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Configure PF4 as a GPIO output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
}

/**
 * @brief Sets the state of the Door Lock LED connected to PF4.
 *
 * @param status 1 to turn LED ON, 0 to turn LED OFF.
 */
void setDoorLockLed(int status) {
    switch (status) {
        case 1: // Turn LED ON
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
            break;
        case 0: // Turn LED OFF
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
            break;
        default: // Optional: handle invalid status, e.g., turn LED OFF
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
            break;
    }
}

//*****************************************************************************
//
// Buzzer Control (GPIO Port F)
//
//*****************************************************************************

/**
 * @brief Initializes GPIO Pin PF3 for Buzzer control.
 *
 * Configures PF3 as a GPIO output.
 */
void initBuzzer() {
    // Enable the clock for GPIO Port F (may already be enabled)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Configure PF3 as a GPIO output for the buzzer.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); // Ensure buzzer is off initially
}

/**
 * @brief Turns the buzzer ON.
 *
 * Sets the buzzer control pin (PF3) HIGH.
 */
void setOnBuzzer() {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

/**
 * @brief Turns the buzzer OFF.
 *
 * Sets the buzzer control pin (PF3) LOW.
 */
void setOffBuzzer() {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
}

/**
 * @brief Toggles the buzzer at a specified frequency (half-period).
 *
 * If `frequency_ms_half_period` is greater than 0, the buzzer is turned ON for
 * `frequency_ms_half_period` milliseconds, then OFF for the same duration.
 * This creates a square wave. To generate a continuous tone,
 * PWM (Pulse Width Modulation) would be more appropriate.
 *
 * @param frequency_ms_half_period The duration in milliseconds for the ON state
 * and for the OFF state (half of the period).
 * If 0 or less, the function does nothing.
 */
void setBuzzerFrequency(int frequency_ms_half_period) {
    if (frequency_ms_half_period > 0){
        setOnBuzzer();
        delay_ms(frequency_ms_half_period); // Buzzer ON for this duration
        setOffBuzzer();
        delay_ms(frequency_ms_half_period); // Buzzer OFF for this duration
    }
}

//*****************************************************************************
//
// System State Checkers
//
//*****************************************************************************

/**
 * @brief Checks if the current gear state is DRIVE.
 * @return True if gearState is GEAR_DRIVE, false otherwise.
 */
bool isGearDrive()   { return gearState == GEAR_DRIVE; }

/**
 * @brief Checks if the current gear state is REVERSE.
 * @return True if gearState is GEAR_REVERSE, false otherwise.
 */
bool isGearReverse() { return gearState == GEAR_REVERSE; }

/**
 * @brief Checks if the ignition is ON.
 * @return True if ignitionState is non-zero (ON), false otherwise.
 */
bool isIgnitionOn()  { return ignitionState != 0; } // Or `return ignitionState;` if 1 means ON

/**
 * @brief Checks if the driver's door is open.
 * @return True if driverDoorState is non-zero (OPEN), false otherwise.
 */
bool isDriverDoorOpen() { return driverDoorState != 0; } // Or `return driverDoorState;` if 1 means OPEN

//*****************************************************************************
//
// Door Locking Control Logic
//
//*****************************************************************************

/**
 * @brief Activates the door locking mechanism.
 *
 * Currently, this function only turns ON the Red LED (PF1) as an indicator.
 * Actual door locking hardware control would be implemented here.
 */
void lockDoors() {
    // Placeholder: Turn Red LED on (PF1) to indicate "locked"
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

/**
 * @brief Activates the door unlocking mechanism.
 *
 * Currently, this function only turns OFF the Red LED (PF1) as an indicator.
 */
void unlockDoors() {
    // Placeholder: Turn Red LED off (PF1) to indicate "unlocked"
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
}

//*****************************************************************************
//
// Manual Lock/Unlock State Flags
//
//*****************************************************************************

/**
 * @brief Checks if the manual lock lever is currently activated.
 *
 * This state is updated by the GPIOA_Handler ISR.
 *
 * @return True if the manual lock lever is pressed/activated, false otherwise.
 */
bool isManualLockOn(void) {
    return manualLockState;
}

/**
 * @brief Checks if the manual unlock lever is currently activated.
 *
 * This state is updated by the GPIOA_Handler ISR.
 *
 * @return True if the manual unlock lever is pressed/activated, false otherwise.
 */
bool isManualUnlockOn(void) {
    return manualUnlockState;
}