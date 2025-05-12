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
#include "Drivers/inc/hw_gpio.h"
#include "Drivers/inc/hw_ints.h"
#include "Drivers/inc/hw_memmap.h"


#define TIMER0A_PERIPH SYSCTL_PERIPH_TIMER0
#define TIMER0A_BASE TIMER0_BASE
#define TIMER0A_TIMER TIMER_A
#define TIMER1A_PERIPH SYSCTL_PERIPH_TIMER1
#define TIMER1A_BASE TIMER1_BASE
#define TIMER1A_TIMER TIMER_A
#define TIMER1A_INT INT_TIMER1A
#define TIMER1A_EVENT TIMER_CAPA_EVENT
#define TIMER1A_GPIO_PERIPH SYSCTL_PERIPH_GPIOB
#define TIMER1A_GPIO_PORT GPIO_PORTB_BASE
#define TIMER1A_GPIO_PIN GPIO_PIN_4
#define TIMER1A_GPIO_PIN_CONFIG GPIO_PB4_T1CCP0

#define ULTRASONIC_TICKS_TO_US (SysCtlClockGet() / 1000000) // 1 microsecond in clock ticks

#define ULTRASONIC_PERIPH_TRIGGER SYSCTL_PERIPH_GPIOE
#define ULTRASONIC_PORT_TRIGGER GPIO_PORTE_BASE
#define ULTRASONIC_TRIGGER_PIN GPIO_PIN_4

#define ULTRASONIC_TICK_TIME (1/SysCtlClockGet())

volatile uint32_t risingEdge = 0;
volatile uint32_t fallingEdge = 0;
volatile bool captured = false;

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
    volatile unsigned int i, j;
    for (i = 0; i < ms; i++)
    {
        for (j = 0; j < 1334; j++)  // ~1ms delay at 16MHz
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

    // Setup interrupt on PA2?PA7
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

// Read the speed from the ADC (scaling from 0?4095 to 0?120 km/h)
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

    // Convert the ADC result (0?4095) to speed (0?120 km/h)
    int speed = (result * 120) / 4095;
    return speed;
}

void initTimer0(void) {
    SysCtlPeripheralEnable(TIMER0A_PERIPH); // Enable clock to Timer0
    while (!SysCtlPeripheralReady(TIMER0A_PERIPH)); // Wait for Timer0 to be ready
    TimerDisable(TIMER0A_BASE, TIMER0A_TIMER); // Disable Timer0A before configuration

    TimerConfigure(TIMER0A_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT); // Configure Timer0A as one-shot timer
}
void timer0_delay(uint32_t time) {
    TimerLoadSet(TIMER0A_BASE, TIMER0A_TIMER, time * 16 - 1); // Load the timer with the delay value (in microseconds)
    TimerIntClear(TIMER0A_BASE, TIMER_TIMA_TIMEOUT); // Clear any pending interrupts
    TimerEnable(TIMER0A_BASE, TIMER0A_TIMER); // Enable Timer0A

    while ((TimerIntStatus(TIMER0A_BASE, false) & TIMER_TIMA_TIMEOUT) == 0); // Wait for the timeout interrupt
}

void initTimer1(void (*handler)(void)) {
    SysCtlPeripheralEnable(TIMER1A_PERIPH); // Enable clock to Timer1A
    while (!SysCtlPeripheralReady(TIMER1A_PERIPH)); // Wait for Timer1A to be ready
    SysCtlPeripheralEnable(TIMER1A_GPIO_PERIPH); // Enable clock to GPIOB for TIMER1A
    while (!SysCtlPeripheralReady(TIMER1A_GPIO_PERIPH)); // Wait for GPIOB to be ready

    GPIOPinTypeTimer(TIMER1A_GPIO_PORT, TIMER1A_GPIO_PIN); // Set PB4 as TIMER1A input
    GPIOPinConfigure(TIMER1A_GPIO_PIN_CONFIG); // Configure PB4 for TIMER1A capture

    TimerDisable(TIMER1A_BASE, TIMER1A_TIMER); // Disable TIMER1A before configuration
    TimerConfigure(TIMER1A_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP); // Configure TIMER1A as capture timer
    TimerLoadSet(TIMER1A_BASE, TIMER1A_TIMER, 0xFFFF); // Load TIMER1A with maximum value
    TimerControlEvent(TIMER1A_BASE, TIMER1A_TIMER, TIMER_EVENT_BOTH_EDGES); // Configure TIMER1A for both edges capture
    
    IntRegister(TIMER1A_INT, handler); // Register the interrupt handler for TIMER1A
    IntEnable(TIMER1A_INT); // Enable the interrupt for TIMER1A
    TimerIntClear(TIMER1A_BASE, TIMER1A_EVENT); // Clear any pending interrupts for TIMER1A
    TimerIntEnable(TIMER1A_BASE, TIMER1A_EVENT); // Enable the capture event interrupt for TIMER1A
}
void timer1_enable(void) {
    TimerEnable(TIMER1A_BASE, TIMER1A_TIMER); // Enable TIMER1A
}
void timer1_disable(void) {
    TimerDisable(TIMER1A_BASE, TIMER1A_TIMER); // Disable TIMER1A
}
void timer1_clear(void) {
    TimerIntClear(TIMER1A_BASE, TIMER1A_EVENT); // Clear the capture event interrupt for TIMER1A
}

void ultrasonic_init(void) {
    SysCtlPeripheralEnable(ULTRASONIC_PERIPH_TRIGGER);
    while (!SysCtlPeripheralReady(ULTRASONIC_PERIPH_TRIGGER));

    GPIOPinTypeGPIOOutput(ULTRASONIC_PORT_TRIGGER, ULTRASONIC_TRIGGER_PIN);
    initTimer0(); // Initialize Timer0 for delay
    initTimer1(ultrasonic_edge_processor); // Initialize Timer1 for edge detection
}

void ultrasonic_trigger(void) {
    GPIOPinWrite(ULTRASONIC_PORT_TRIGGER, ULTRASONIC_TRIGGER_PIN, ULTRASONIC_TRIGGER_PIN);
    timer0_delay(10); // 10 microseconds pulse width
    GPIOPinWrite(ULTRASONIC_PORT_TRIGGER, ULTRASONIC_TRIGGER_PIN, 0);
}

uint32_t ultrasonic_get_distance(void) {
    
    timer1_enable(); // Enable Timer1 for edge detection
    ultrasonic_trigger(); // Send trigger pulse
    while (!captured);
    timer1_disable(); // Disable Timer1 after measurement
    timer1_clear(); // Clear Timer1 interrupt flag

    float timeDiff;
    if (fallingEdge >= risingEdge) {
        timeDiff = fallingEdge - risingEdge;
    } else {
        timeDiff = (0xFFFFFFFF - risingEdge) + fallingEdge;
    }
    captured = false;
    
    float distance_cm = timeDiff / 932.9446064;
    
    if (distance_cm > 400) { 
        distance_cm = 400;
    }
    return distance_cm;
}

void ultrasonic_edge_processor(void) {
    TimerIntClear(TIMER1A_BASE, TIMER1A_EVENT); // Clear the interrupt flag

    if (GPIOPinRead(TIMER1A_GPIO_PORT, TIMER1A_GPIO_PIN) == TIMER1A_GPIO_PIN) { // Rising edge
        risingEdge = TimerValueGet(TIMER1A_BASE, TIMER1A_TIMER);
    } else { // Falling edge
        fallingEdge = TimerValueGet(TIMER1A_BASE, TIMER1A_TIMER);
        captured = true; // Set the flag to indicate that the measurement is done
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

void setOnBuzzer() {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void setOffBuzzer() {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
}

void setBuzzerFrequency(int frequency) {
    if (frequency > 0){
        setOnBuzzer();
				delay_ms(frequency);
				setOffBuzzer();
				delay_ms(frequency);
		}
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