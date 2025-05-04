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

volatile int manualLockFlag = 0;
volatile int manualUnlockFlag = 0;
volatile int ignitionState = 0;
volatile int driverDoorState = 0;

void PortA_ISR(void) {
    uint32_t status = GPIOIntStatus(GPIO_PORTA_BASE, true);
    GPIOIntClear(GPIO_PORTA_BASE, status);

    if (status & GPIO_PIN_6) manualLockFlag = 1;
    if (status & GPIO_PIN_7) manualUnlockFlag = 1;
    if (status & GPIO_PIN_4) ignitionState = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == 0;
    if (status & GPIO_PIN_5) driverDoorState = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) == 0;
}

void initGPIO() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Enable interrupts for PA2–PA7
    GPIOIntDisable(GPIO_PORTA_BASE, 0xFC); // Disable all first
    GPIOIntClear(GPIO_PORTA_BASE, 0xFC);
    GPIOIntRegister(GPIO_PORTA_BASE, PortA_ISR);
    GPIOIntTypeSet(GPIO_PORTA_BASE, 0xFC, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTA_BASE, 0xFC);
}

void initADC() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

int readSpeedADC() {
    uint32_t result;
    ADCProcessorTrigger(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false));
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &result);
    return (result * 120) / 4095;
}

void initUltrasonic() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    GPIOPinConfigure(0x00011807); // Hardcoded value for GPIO_PB6_T1CCP0
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);

    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME);
    TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);
    TimerEnable(TIMER1_BASE, TIMER_A);
} 

int measureDistance() {
    uint32_t startTime, endTime;

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    SysCtlDelay(SysCtlClockGet() / (3 * 1000000)); // 10us pulse
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);

    while (!(TimerIntStatus(TIMER1_BASE, true) & TIMER_CAPA_EVENT));
    startTime = TimerValueGet(TIMER1_BASE, TIMER_A);
    TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);

    while (!(TimerIntStatus(TIMER1_BASE, true) & TIMER_CAPA_EVENT));
    endTime = TimerValueGet(TIMER1_BASE, TIMER_A);
    TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);

    uint32_t pulseWidth = (startTime > endTime) ? (startTime - endTime) : (endTime - startTime);
    int distance = (pulseWidth * 0.017) / 2; // in cm assuming 16MHz clock
    return distance;
}

void initBuzzer() {
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
}

void setBuzzerFrequency(int frequency) {
    if (frequency > 0)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    else
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
}

void initRGB() {
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void setRGBColor(char color) {
    switch(color) {
        case 'R': GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1); break;
        case 'G': GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3); break;
        case 'Y': GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1 | GPIO_PIN_3); break;
        default:  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0); break;
    }
}

int isGearDrive() {
    return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == 0;
}

int isGearReverse() {
    return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) == 0;
}

int isIgnitionOn() {
    return ignitionState;
}

int isDriverDoorOpen() {
    return driverDoorState;
}

void lockDoors() {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); // Red LED as lock indicator
}

void unlockDoors() {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); // Turn off Red LED
}

int isManualLockPressed() {
    if (manualLockFlag) {
        manualLockFlag = 0;
        return 1;
    }
    return 0;
}

int isManualUnlockPressed() {
    if (manualUnlockFlag) {
        manualUnlockFlag = 0;
        return 1;
    }
    return 0;
}
