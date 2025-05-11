#include "drivers.h"
#include "tm4c123gh6pm.h"
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
void initGPIO() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA) ||
           !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Output (RGB + buzzer + doorLockLed)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

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

// Ultrasonic Sensor (PB5 trigger, PB6 echo)
void initUltrasonic() {
    // 1) Enable the GPIOB and Timer1 peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB) ||
           !SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));

    // 2) PB5 = trigger pin (GPIO output)
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

    // 3) PB6 = Timer1A capture input (use the standard macro)
    GPIOPinConfigure(GPIO_PIN_6);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);

    // 4) Configure Timer1A as a split 16-bit timer in A-capture mode
    TimerDisable(TIMER1_BASE, TIMER_A);
    TimerConfigure(TIMER1_BASE,
                   TIMER_CFG_SPLIT_PAIR    // two 16-bit timers
                   | TIMER_CFG_A_CAP_TIME  // Timer A = edge-capture
                   );
    // Capture on BOTH edges
    TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);

    // 5) Clear any leftover event, then enable the capture interrupt
    TimerIntClear (TIMER1_BASE, TIMER_CAPA_EVENT);
    TimerIntEnable(TIMER1_BASE, TIMER_CAPA_EVENT);

    // 6) Finally, enable the timer
    TimerEnable(TIMER1_BASE, TIMER_A);
 
}

int measureDistance() {
    uint32_t startTime, endTime;

    // Trigger pulse from PB5
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    SysCtlDelay(SysCtlClockGet() / (3 * 1000000));  // ~10 us pulse
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);

    // Switch PB6 back to timer
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);

// --- clear any old capture flags ---
    TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);

    // wait for first edge (with ~20 ms timeout)
    {
      // wait for first edge (with ~20 ms timeout)
			uint32_t loops = (SysCtlClockGet() * 20) / 1000;
      while (!(TimerIntStatus(TIMER1_BASE, true) & TIMER_CAPA_EVENT)) {
        if (--loops == 0) {
          // no echo—return max distance
          return 255;
        }
      }
      startTime = TimerValueGet(TIMER1_BASE, TIMER_A);
      TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);
    }

    // wait for second edge (with ~20 ms timeout)
    {
      // wait for first edge (with ~20 ms timeout)
			uint32_t loops = (SysCtlClockGet() * 20) / 1000;
      while (!(TimerIntStatus(TIMER1_BASE, true) & TIMER_CAPA_EVENT)) {
        if (--loops == 0) {
          return 255;
        }
      }
      endTime = TimerValueGet(TIMER1_BASE, TIMER_A);
      TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);
    }

    uint32_t pulseWidth = (startTime > endTime) ? (startTime - endTime) : (endTime - startTime);
    float time_us = pulseWidth * 62.5f; // 62.5 ns ticks
    return (int)((time_us / 2.0f) * 0.0343f);  // cm
}

// RGB LED
void initRGB() {
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

// Buzzer
void initBuzzer() {
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