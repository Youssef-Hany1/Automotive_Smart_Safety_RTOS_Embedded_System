#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <stdint.h>
#include <stdbool.h>
// Include necessary TivaWare driver library headers
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h" // For GPIO pin muxing definitions

// --- Constants ---
#define SPEED_THRESHOLD 10      // Speed threshold in km/h for auto-lock
#define SAFE_ZONE_DISTANCE 100  // Distance in cm
#define CAUTION_ZONE_DISTANCE 30 // Distance in cm

// --- Initialization Function Prototypes ---
void ADC_Init(void);
void Ultrasonic_Timer_Init(void);
void LCD_Init(void);
void Buzzer_Init(void);
void RGB_LED_Init(void);
void Lock_Mechanism_Init(void);
void Input_Switches_Init(void); // Initializes Gear, Door, Manual, Ignition pins

// --- Peripheral Control Function Prototypes ---
uint32_t Read_Potentiometer(void);
void Trigger_Ultrasonic(void);
uint32_t Read_Ultrasonic_Echo_Timer(void); // Needs careful implementation or ISR
int Calculate_Speed(uint32_t adcValue);
int Calculate_Distance(uint32_t timerValue);
void Lock_Doors(void);
void Unlock_Doors(void);
void Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue);
void Activate_Buzzer(uint32_t frequency); // Note: Simple implementation ignores frequency
void Deactivate_Buzzer(void);

// --- LCD Function Prototypes ---
void LCD_Display_Status(const char *status);
void LCD_Display_Speed(int speed);
void LCD_Display_Distance(int distance);
void LCD_Display_Message(const char *message); // Displays on a specific line (e.g., line 0)
void LCD_Clear(void);

#endif // PERIPHERALS_H
