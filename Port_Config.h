#ifndef PORT_CONFIG_H
#define PORT_CONFIG_H

#include "driverlib/sysctl.h" // For SYSCTL_PERIPH_GPIOx defines
#include "driverlib/gpio.h"   // For GPIO_PIN_x defines
#include "inc/hw_memmap.h"    // For GPIO_PORTx_BASE defines

// --- USER CONFIGURATION REQUIRED ---
// Replace the example ports and pins below with your actual hardware connections.

// --- Potentiometer (Speed Sensor) ---
#define POT_ADC_PERIPH      SYSCTL_PERIPH_ADC0
#define POT_GPIO_PERIPH     SYSCTL_PERIPH_GPIOE
#define POT_GPIO_PORT       GPIO_PORTE_BASE
#define POT_GPIO_PIN        GPIO_PIN_3       // Example: PE3/AIN0
#define POT_ADC_BASE        ADC0_BASE
#define POT_ADC_SEQUENCER   3
#define POT_ADC_CHANNEL     ADC_CTL_CH0      // Matches AIN0

// --- Ultrasonic Sensor ---
#define ULTRA_TRIG_GPIO_PERIPH SYSCTL_PERIPH_GPIOB
#define ULTRA_TRIG_PORT        GPIO_PORTB_BASE
#define ULTRA_TRIG_PIN         GPIO_PIN_4       // Example: PB4 for Trigger

#define ULTRA_ECHO_GPIO_PERIPH SYSCTL_PERIPH_GPIOC
#define ULTRA_ECHO_PORT        GPIO_PORTC_BASE
#define ULTRA_ECHO_PIN         GPIO_PIN_4       // Example: PC4 for Echo (Needs Timer Capture Pin)
#define ULTRA_ECHO_TIMER_PERIPH SYSCTL_PERIPH_WTIMER0
#define ULTRA_ECHO_TIMER_BASE  WTIMER0_BASE
#define ULTRA_ECHO_TIMER_PIN_CONFIG GPIO_PC4_WT0CCP0 // Example Timer Capture Pin Mux
#define ULTRA_ECHO_TIMER       TIMER_A          // Example: Using Timer A of WTIMER0
#define ULTRA_TIMER_PRESCALER  16               // Example: 16MHz clock / 16 = 1MHz timer -> 1us resolution

// --- Gear Shift Switches ---
#define GEAR_GPIO_PERIPH    SYSCTL_PERIPH_GPIOD
#define GEAR_PORT           GPIO_PORTD_BASE
#define PARK_PIN            GPIO_PIN_0       // Example: PD0
#define REVERSE_PIN         GPIO_PIN_1       // Example: PD1
#define DRIVE_PIN           GPIO_PIN_2       // Example: PD2
// Define interrupt base if using interrupts for gears
#define GEAR_INT_BASE       INT_GPIOD

// --- Driver Door Switch ---
#define DOOR_GPIO_PERIPH    SYSCTL_PERIPH_GPIOD
#define DOOR_PORT           GPIO_PORTD_BASE
#define DOOR_PIN            GPIO_PIN_3       // Example: PD3
// Define interrupt base if using interrupts for door
#define DOOR_INT_BASE       INT_GPIOD

// --- Manual Lock/Unlock Buttons ---
#define MANUAL_CTRL_GPIO_PERIPH SYSCTL_PERIPH_GPIOF
#define MANUAL_CTRL_PORT        GPIO_PORTF_BASE
#define MANUAL_LOCK_PIN         GPIO_PIN_4       // Example: PF4 (SW1 on Launchpad)
#define MANUAL_UNLOCK_PIN       GPIO_PIN_0       // Example: PF0 (SW2 on Launchpad)
// Define interrupt base if using interrupts for manual controls
#define MANUAL_CTRL_INT_BASE    INT_GPIOF

// --- Ignition Switch ---
#define IGNITION_GPIO_PERIPH SYSCTL_PERIPH_GPIOD
#define IGNITION_PORT        GPIO_PORTD_BASE
#define IGNITION_PIN         GPIO_PIN_6       // Example: PD6
// Define interrupt base if using interrupts for ignition
#define IGNITION_INT_BASE    INT_GPIOD

// --- Door Lock Mechanism ---
#define LOCK_GPIO_PERIPH    SYSCTL_PERIPH_GPIOA
#define LOCK_PORT           GPIO_PORTA_BASE
#define LOCK_PIN            GPIO_PIN_2       // Example: PA2
#define UNLOCK_PIN          GPIO_PIN_3       // Example: PA3

// --- RGB LED ---
#define LED_GPIO_PERIPH     SYSCTL_PERIPH_GPIOF
#define LED_PORT            GPIO_PORTF_BASE
#define LED_RED_PIN         GPIO_PIN_1       // Example: PF1 (On-board Launchpad LED)
#define LED_GREEN_PIN       GPIO_PIN_3       // Example: PF3 (On-board Launchpad LED)
#define LED_BLUE_PIN        GPIO_PIN_2       // Example: PF2 (On-board Launchpad LED)

// --- Buzzer ---
#define BUZZER_GPIO_PERIPH  SYSCTL_PERIPH_GPIOA
#define BUZZER_PORT         GPIO_PORTA_BASE
#define BUZZER_PIN          GPIO_PIN_4       // Example: PA4

// --- LCD Display ---
// Data Lines (Example: PB0-PB3 for D4-D7)
#define LCD_DATA_GPIO_PERIPH SYSCTL_PERIPH_GPIOB
#define LCD_DATA_PORT        GPIO_PORTB_BASE
#define LCD_D4_PIN           GPIO_PIN_0       // Example: PB0 -> LCD D4
#define LCD_D5_PIN           GPIO_PIN_1       // Example: PB1 -> LCD D5
#define LCD_D6_PIN           GPIO_PIN_2       // Example: PB2 -> LCD D6
#define LCD_D7_PIN           GPIO_PIN_3       // Example: PB3 -> LCD D7
// Control Lines (Example: PA5=RS, PA6=EN)
#define LCD_CTRL_GPIO_PERIPH SYSCTL_PERIPH_GPIOA
#define LCD_CTRL_PORT        GPIO_PORTA_BASE
#define LCD_RS_PIN           GPIO_PIN_5       // Example: PA5 -> LCD RS
#define LCD_EN_PIN           GPIO_PIN_6       // Example: PA6 -> LCD EN

#endif // PORT_CONFIG_H
