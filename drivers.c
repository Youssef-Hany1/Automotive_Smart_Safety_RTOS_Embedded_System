#include "drivers.h"
#include "tm4c123gh6pm.h"
#include "TM4C123.h"
#include <stdint.h>
#include <stdbool.h>
#include "Drivers/driverlib/sysctl.h" 
#include "Drivers/driverlib/gpio.h"
#include "Drivers/driverlib/adc.h"
#include "Drivers/driverlib/pin_map.h"
#include "Drivers/driverlib/timer.h"
#include "Drivers/driverlib/interrupt.h"
#include "Drivers/inc/hw_ints.h"
#include "Drivers/inc/hw_memmap.h"


#define TRIGGER_PIN GPIO_PIN_5   // Trigger pin (PB5)
#define ECHO_PIN GPIO_PIN_4      // Echo pin (PB4)

volatile uint32_t startTime, endTime, pulseWidth;
volatile bool pulseCaptured = false;

// Global volatile flags
volatile int ignitionState = 0;
volatile int driverDoorState = 0;
//latch for gear switch
volatile Gear_t gearState = GEAR_NEUTRAL;
volatile bool manualLockState     = false;
volatile bool manualUnlockState   = false;

// Debounce delay (~1 ms)
void debounceDelay() {
    SysCtlDelay(SysCtlClockGet() / 3000);
}

void delay_ms(uint32_t ms)
{
	volatile uint32_t i;
	while (ms--)
	{
		for (i = 0; i < 4000; i++)
		{
		}
	}
}

// GPIO Port A ISR handler
void GPIOA_Handler(void) {
    uint32_t status = GPIOIntStatus(GPIO_PORTA_BASE, true);
    GPIOIntClear(GPIO_PORTA_BASE, status);
    debounceDelay();

    // PA6 = lock lever: high when up, low when down
    if (status & GPIO_PIN_6)
        manualLockState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) & GPIO_PIN_6) != 0;

    // PA7 = unlock lever
    if (status & GPIO_PIN_7)
        manualUnlockState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) & GPIO_PIN_7) != 0;
 

    if (status & GPIO_PIN_4) ignitionState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == 0);
    if (status & GPIO_PIN_5) driverDoorState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) == 0);
	
		// latch gear state on each button press
    // --- DRIVE lever (PA2) ---
    if (status & GPIO_PIN_2) {
        // reads 0 when switch closed to ground
        if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == 0)
            gearState = GEAR_DRIVE;      // lever down ? DRIVE
        else
            gearState = GEAR_NEUTRAL;    // lever up   ? back to NEUTRAL
    }

    // --- REVERSE lever (PA3) ---
    if (status & GPIO_PIN_3) {
        if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) == 0)
            gearState = GEAR_REVERSE;    // lever down ? REVERSE
        else
            gearState = GEAR_NEUTRAL;    // lever up   ? NEUTRAL
    }

    // --- IGNITION lever (PA4) ---
    if (status & GPIO_PIN_4) {
        ignitionState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == 0);
    }

    // --- DRIVER-DOOR lever (PA5) ---
    if (status & GPIO_PIN_5) {
        driverDoorState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) == 0);
    }

    // manual-lock lever (PA6)
    if (status & GPIO_PIN_6)
        manualLockState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0);
		 
    // manual-unlock lever (PA7)
    if (status & GPIO_PIN_7)
        manualUnlockState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) == 0);
		 
}

// GPIO Initialization
void initSwitches() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    // Input (manual lock/unlock, ignition, gear, door)
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Setup interrupt on PA2–PA7
    GPIOIntDisable(GPIO_PORTA_BASE, 0xFC);
    GPIOIntClear(GPIO_PORTA_BASE, 0xFC);
    GPIOIntRegister(GPIO_PORTA_BASE, GPIOA_Handler);
    GPIOIntTypeSet(GPIO_PORTA_BASE, 0xFC, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTA_BASE, 0xFC);
}

// ADC (Speed input from PE3 - Channel 0)
void initADC() {
    // Enable the peripherals for ADC0 and GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Wait for the peripherals to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    // Configure PE3 as ADC input
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // Configure the ADC sequence (Sequence 3 in this case)
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); // Triggered by processor
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END); // Channel 0, Interrupt enabled, End of sequence

    // Enable the ADC sequence and clear any interrupt flags
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

// Read the speed from the ADC (scaling from 0–4095 to 0–120 km/h)
int readSpeedADC() {
    uint32_t result;
    
    // Trigger the ADC conversion
    ADCProcessorTrigger(ADC0_BASE, 3);

    // Wait for the conversion to finish
    while (!ADCIntStatus(ADC0_BASE, 3, false));

    // Clear the interrupt flag
    ADCIntClear(ADC0_BASE, 3);

    // Retrieve the result from the ADC
    ADCSequenceDataGet(ADC0_BASE, 3, &result);

    // Convert the ADC result (0–4095) to speed (0–120 km/h)
    int speed = (result * 120) / 4095;
    return speed;
}

// Initialize the PortB and Timer for the Ultrasonic Sensor
void initPortB(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // Configure PB5 (Trigger) as output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, TRIGGER_PIN);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ECHO_PIN);
    GPIOPinConfigure(0x00000404);  // Configure PB4 as T1CCP0 (Timer1 Capture Pin)
}

// Initialize Timer1 to capture the pulse width
void initTimer1(void) {
    // Enable Timer1 and GPIOB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // Configure PB4 (T1CCP0) for Timer input capture
    GPIOPinConfigure(0x00000404);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, ECHO_PIN);  // Configure as Timer pin

    // Configure Timer1 for capture mode
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_CAP_TIME); // Capture mode (edge time)
    TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);  // Both rising and falling edges

    // Set the timer to 16-bit and start with the maximum possible value
    TimerLoadSet(TIMER1_BASE, TIMER_A, 0xFFFF);
    TimerEnable(TIMER1_BASE, TIMER_A);  // Enable Timer1
}

// Initialize ultrasonic sensor
void initUltrasonic(void) {
    initPortB();
    initTimer1();
}

// Trigger the Ultrasonic sensor to send a pulse
void triggerUltrasonicSensor(void) {
    GPIOPinWrite(GPIO_PORTB_BASE, TRIGGER_PIN, TRIGGER_PIN); // Set trigger high
    SysCtlDelay(160);  // Delay to keep the trigger high for 10µs
    GPIOPinWrite(GPIO_PORTB_BASE, TRIGGER_PIN, 0); // Set trigger low
}

// Timer interrupt handler to capture the pulse width
void TIMER1A_Handler(void) {
    if (TimerIntStatus(TIMER1_BASE, true) & TIMER_TIMA_TIMEOUT) {
        // Capture the start and end times for the echo pulse
        if (GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN)) {
            // Rising edge - capture start time
            startTime = TimerValueGet(TIMER1_BASE, TIMER_A);
        } else {
            // Falling edge - capture end time and calculate pulse width
            endTime = TimerValueGet(TIMER1_BASE, TIMER_A);
            if (endTime >= startTime) {
                pulseWidth = endTime - startTime;
            } else {
                pulseWidth = (0xFFFF - startTime) + endTime;  // Handle rollover
            }
            pulseCaptured = true;  // Set flag to indicate pulse captured
        }

        // Clear the interrupt flag
        TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    }
}

// Function to read the ultrasonic sensor distance
uint32_t ultrasonicReadValue(void) {
    triggerUltrasonicSensor();  // Trigger ultrasonic sensor pulse
    pulseCaptured = false;      // Reset flag before capturing pulse

    // Wait for the capture to complete (non-blocking)
    uint32_t timeout = 1000;  // Timeout after 1000ms
    while (!pulseCaptured && timeout > 0) {
        timeout--;
    }

    if (pulseCaptured) {
        // Calculate distance in cm using the pulse width
        float distance = (pulseWidth * 0.0343) / 2.0;  // Speed of sound is ~343 m/s
        return (uint32_t)distance;
    } else {
        return 0;  // Timeout or error
    }
}

// RGB LED
void initRGB() {
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Output (RGB)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0);
}

void setRGBColor(char color) {
    switch (color) {
        case 'R': GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_1); break;
        case 'G': GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_2); break;
        case 'Y': GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_1 | GPIO_PIN_2); break;
        default:  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0); break;
    }
}

void initDoorLockLed(void) {
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
	
    // Output (DoorLockLed)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
}

void setDoorLockLed(int status) {
		switch (status) {
        case 1: GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); break;
        case 0:  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); break;
    }
}

// Buzzer
void initBuzzer() {
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Output (buzzer)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
}

void setBuzzerFrequency(int frequency) {
    if (frequency > 0){
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
				delay_ms(frequency);
			  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
			delay_ms(frequency);
		} else
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
}

// System State Checkers
bool isGearDrive()   { return gearState == GEAR_DRIVE; }
bool isGearReverse() { return gearState == GEAR_REVERSE; }
bool isIgnitionOn()  { return ignitionState; }
bool isDriverDoorOpen() { return driverDoorState; }

// Locking Control
void lockDoors() {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);  // Red LED on
}

void unlockDoors() {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);  // Red LED off
}

// Manual Lock/Unlock Flags
bool isManualLockOn(void) {
    return manualLockState;
}

bool isManualUnlockOn(void) {
    return manualUnlockState;
}