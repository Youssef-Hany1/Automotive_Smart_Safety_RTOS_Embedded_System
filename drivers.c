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


#define TRIGGER_PIN (1 << 5) // PB5

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

// ADC (Speed input from PE3 - channel 0)
void initADC() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

int readSpeedADC() {
    uint32_t result;
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false));
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &result);
    return (result * 120) / 4095; // Scaled to 0–120 km/h
} 

void initPortA(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    GPIOB->DIR |= TRIGGER_PIN;    // Set PB5 as output
    GPIOB->DEN |= TRIGGER_PIN;    // Enable digital on PB5
    GPIOB->AFSEL &= ~TRIGGER_PIN; // Disable alternate function
    GPIOB->PCTL &= ~(0x0F << 24); // Clear PCTL for PB5
    GPIOB->AMSEL &= ~TRIGGER_PIN; // Disable analog on PB5
}

void initTimer1(void)
{
    // 1. Enable clock for Timer1 and GPIOB
    SYSCTL->RCGCTIMER |= (1 << 1); // Enable Timer1 clock
    SYSCTL->RCGCGPIO |= (1 << 1);  // Enable GPIOB clock
    while ((SYSCTL->PRTIMER & (1 << 1)) == 0)
        ; // Wait for Timer1 ready
    while ((SYSCTL->PRGPIO & (1 << 1)) == 0)
        ; // Wait for GPIOB ready

    // 2. Configure PB4 (T1CCP0) for timer input capture
    GPIOB->DIR &= ~(1 << 4);    // PB4 as input
    GPIOB->AFSEL |= (1 << 4);   // Enable alternate function on PB4
    GPIOB->PCTL &= ~0x000F0000; // Clear PCTL bits for PB4
    GPIOB->PCTL |= 0x00070000;  // Configure PB4 as T1CCP0
    GPIOB->DEN |= (1 << 4);     // Enable digital function on PB4

    // 3. Disable Timer1A before configuration
    TIMER1->CTL &= ~(1 << 0); // Disable Timer1A

    // 4. Configure Timer1A for 16-bit, capture mode
    TIMER1->CFG = 0x00000004; // 16-bit timer configuration
    TIMER1->TAMR = 0x17;      // Timer A Mode: Capture | Edge-Time | Count-Up

    // 5. Capture on both edges (rising and falling)
    TIMER1->CTL &= ~(0x3 << 2); // Clear TAEVENT bits
    TIMER1->CTL |= (0x3 << 2);  // Both edges

    // 6. Set timer interval (not required in edge-time mode, but good practice)
    TIMER1->TAILR = 0xFFFF; // Max count for 16-bit

    // 7. Clear any pending interrupts and optionally enable capture interrupt
    TIMER1->ICR = (1 << 2); // Clear capture event interrupt

    // Optional: Enable interrupt (commented out if polling is used)
    // TIMER1->IMR |= (1 << 2);             // Unmask capture match interrupt
    // NVIC_EnableIRQ(TIMER1A_IRQn);       // Enable IRQ in NVIC

    // 8. Enable Timer1A
    TIMER1->CTL |= (1 << 0); // Enable Timer1A
}

void initUltrasonic(void)
{
    initPortA();  // Trigger pin (PA6)
    initTimer1(); // Echo pin (PB4 with edge-time capture)
}

uint32_t ultrasonicReadValue()
{
    uint32_t risingEdge, fallingEdge, pulseWidth;
    float distance;

    // Send trigger pulse: 10us HIGH
    GPIOA->DATA &= ~TRIGGER_PIN; // Clear trigger
    for (volatile int i = 0; i < 100; i++);                       // Small delay
    GPIOA->DATA |= TRIGGER_PIN; // Set trigger high
    for (volatile int i = 0; i < 160; i++);                        // Approx 10us delay
    GPIOA->DATA &= ~TRIGGER_PIN; // Set trigger low

    // Wait for rising edge
    while ((TIMER1->RIS & (1 << 2)) == 0);
    risingEdge = TIMER1->TAR;
    TIMER1->ICR |= (1 << 2); // Clear capture flag

    // Wait for falling edge
    while ((TIMER1->RIS & (1 << 2)) == 0);
    fallingEdge = TIMER1->TAR;
    TIMER1->ICR |= (1 << 2); // Clear capture flag

    // Handle overflow
    if (fallingEdge > risingEdge)
        pulseWidth = fallingEdge - risingEdge;
    else
        pulseWidth = (0xFFFF - risingEdge) + fallingEdge;

    // Calculate distance in cm (Time in clock ticks at 16 MHz ? 1 tick = 62.5ns)
    distance = (pulseWidth * 0.0343) / 2; // Speed of sound is ~343 m/s

    return (uint32_t)distance;
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