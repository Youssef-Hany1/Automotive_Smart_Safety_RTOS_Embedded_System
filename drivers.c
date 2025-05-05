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
volatile int manualLockFlag = 0;
volatile int manualUnlockFlag = 0;
volatile int ignitionState = 0;
volatile int driverDoorState = 0;

// Debounce delay (~1 ms)
void debounceDelay() {
    SysCtlDelay(SysCtlClockGet() / 3000);
}

// GPIO Port A ISR handler
void PortA_ISR(void) {
    uint32_t status = GPIOIntStatus(GPIO_PORTA_BASE, true);
    GPIOIntClear(GPIO_PORTA_BASE, status);
    debounceDelay();

    if (status & GPIO_PIN_6) manualLockFlag = 1;
    if (status & GPIO_PIN_7) manualUnlockFlag = 1;
    if (status & GPIO_PIN_4) ignitionState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == 0);
    if (status & GPIO_PIN_5) driverDoorState = (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) == 0);
}

// GPIO Initialization
void initGPIO() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA) ||
           !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF) ||
           !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    // Output (RGB + buzzer)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Input (manual lock/unlock, ignition, gear, door)
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Setup interrupt on PA2–PA7
    GPIOIntDisable(GPIO_PORTA_BASE, 0xFC);
    GPIOIntClear(GPIO_PORTA_BASE, 0xFC);
    GPIOIntRegister(GPIO_PORTA_BASE, PortA_ISR);
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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB) || !SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));

    GPIOPinConfigure(0x00011807);  // PB6 as CCP (capture input)
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);

    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME);
    TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);
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

    while (!(TimerIntStatus(TIMER1_BASE, true) & TIMER_CAPA_EVENT));
    startTime = TimerValueGet(TIMER1_BASE, TIMER_A);
    TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);

    while (!(TimerIntStatus(TIMER1_BASE, true) & TIMER_CAPA_EVENT));
    endTime = TimerValueGet(TIMER1_BASE, TIMER_A);
    TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);

    uint32_t pulseWidth = (startTime > endTime) ? (startTime - endTime) : (endTime - startTime);
    float time_us = pulseWidth * 62.5f; // 62.5 ns ticks
    return (int)((time_us / 2.0f) * 0.0343f);  // cm
}

// RGB LED
void initRGB() {
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void setRGBColor(char color) {
    switch (color) {
        case 'R': GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1); break;
        case 'G': GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3); break;
        case 'Y': GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1 | GPIO_PIN_3); break;
        default:  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0); break;
    }
}

// Buzzer
void initBuzzer() {
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
}

void setBuzzerFrequency(int frequency) {
    if (frequency > 0)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    else
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
}

// System State Checkers
bool isGearDrive()   { return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == 0; }
bool isGearReverse() { return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) == 0; }
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
bool isManualLockPressed() {
    if (manualLockFlag) {
        manualLockFlag = 0;
        return true;
    }
    return false;
}

bool isManualUnlockPressed() {
    if (manualUnlockFlag) {
        manualUnlockFlag = 0;
        return true;
    }
    return false;
}
