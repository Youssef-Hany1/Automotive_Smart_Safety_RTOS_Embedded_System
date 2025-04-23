#include "peripherals.h"
#include "Port_Config.h" // Include hardware definitions
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h" // For HWREG
#include <stdio.h>       // For sprintf in LCD functions if needed
#include <string.h>      // For strlen in LCD functions

// --- Internal LCD Helper Function Prototypes ---
static void LCD_Send_Nibble(uint8_t nibble);
static void LCD_Send_Cmd(uint8_t cmd);
static void LCD_Send_Data(uint8_t data);
static void LCD_Puts(const char *str);
static void LCD_GotoXY(uint8_t x, uint8_t y);

// --- Initialization Functions ---

/* Initialize ADC for Potentiometer */
void ADC_Init(void) {
    // Enable clock to ADC module and corresponding GPIO port
    SysCtlPeripheralEnable(POT_ADC_PERIPH);
    SysCtlPeripheralEnable(POT_GPIO_PERIPH);
    while (!SysCtlPeripheralReady(POT_GPIO_PERIPH)); // Wait for GPIO port to be ready

    // Configure Potentiometer pin as ADC input
    GPIOPinTypeADC(POT_GPIO_PORT, POT_GPIO_PIN);

    // Configure ADC sequencer
    ADCSequenceConfigure(POT_ADC_BASE, POT_ADC_SEQUENCER, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(POT_ADC_BASE, POT_ADC_SEQUENCER, 0, POT_ADC_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(POT_ADC_BASE, POT_ADC_SEQUENCER);

    // Clear interrupt status flag
    ADCIntClear(POT_ADC_BASE, POT_ADC_SEQUENCER);
    printf("DEBUG: ADC Initialized\n");
}

/* Initialize Timer for Ultrasonic Echo Capture */
void Ultrasonic_Timer_Init(void) {
    // Enable Timer peripheral
    SysCtlPeripheralEnable(ULTRA_ECHO_TIMER_PERIPH);
    // Enable GPIO Port for Echo pin
    SysCtlPeripheralEnable(ULTRA_ECHO_GPIO_PERIPH);
    while (!SysCtlPeripheralReady(ULTRA_ECHO_GPIO_PERIPH));

    // Configure Echo Pin alternate function for Timer Capture
    GPIOPinConfigure(ULTRA_ECHO_TIMER_PIN_CONFIG);
    GPIOPinTypeTimer(ULTRA_ECHO_PORT, ULTRA_ECHO_PIN);

    // Configure Timer for Input Edge-Time Capture Mode (Both Edges)
    TimerConfigure(ULTRA_ECHO_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);

    // Configure Timer event detection (Both Edges)
    TimerControlEvent(ULTRA_ECHO_TIMER_BASE, ULTRA_ECHO_TIMER, TIMER_EVENT_BOTH_EDGES);

    // Set Timer prescaler
    TimerPrescaleSet(ULTRA_ECHO_TIMER_BASE, ULTRA_ECHO_TIMER, ULTRA_TIMER_PRESCALER);

    printf("DEBUG: Ultrasonic Timer Initialized\n");
    // TimerEnable is usually called just before measurement or via an ISR
}

/* Initialize GPIO for Buzzer */
void Buzzer_Init() {
    SysCtlPeripheralEnable(BUZZER_GPIO_PERIPH);
    while (!SysCtlPeripheralReady(BUZZER_GPIO_PERIPH));
    GPIOPinTypeGPIOOutput(BUZZER_PORT, BUZZER_PIN);
    GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0); // Initially off
    printf("DEBUG: Buzzer Initialized\n");
}

/* Initialize GPIOs for RGB LED */
void RGB_LED_Init() {
    SysCtlPeripheralEnable(LED_GPIO_PERIPH);
    while (!SysCtlPeripheralReady(LED_GPIO_PERIPH));
    GPIOPinTypeGPIOOutput(LED_PORT, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN);
    GPIOPinWrite(LED_PORT, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN, 0); // Initially off
    printf("DEBUG: RGB LED Initialized\n");
}

/* Initialize GPIOs for Lock Mechanism */
void Lock_Mechanism_Init() {
    SysCtlPeripheralEnable(LOCK_GPIO_PERIPH);
    while (!SysCtlPeripheralReady(LOCK_GPIO_PERIPH));
    GPIOPinTypeGPIOOutput(LOCK_PORT, LOCK_PIN | UNLOCK_PIN);
    GPIOPinWrite(LOCK_PORT, LOCK_PIN | UNLOCK_PIN, 0); // Initially off
    printf("DEBUG: Lock Mechanism Initialized\n");
}

/* Initialize GPIOs for Input Switches/Buttons */
void Input_Switches_Init() {
    // Gear Switches
    SysCtlPeripheralEnable(GEAR_GPIO_PERIPH);
    while (!SysCtlPeripheralReady(GEAR_GPIO_PERIPH));
    GPIOPinTypeGPIOInput(GEAR_PORT, PARK_PIN | REVERSE_PIN | DRIVE_PIN);
    // Add pull-ups/downs if needed, e.g., GPIOPadConfigSet(GEAR_PORT, PARK_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Door Switch
    SysCtlPeripheralEnable(DOOR_GPIO_PERIPH); // Might be same as GEAR port
    while (!SysCtlPeripheralReady(DOOR_GPIO_PERIPH));
    GPIOPinTypeGPIOInput(DOOR_PORT, DOOR_PIN);
    // Add pull-up/down if needed

    // Manual Control Buttons
    SysCtlPeripheralEnable(MANUAL_CTRL_GPIO_PERIPH);
    while (!SysCtlPeripheralReady(MANUAL_CTRL_GPIO_PERIPH));
    GPIOPinTypeGPIOInput(MANUAL_CTRL_PORT, MANUAL_LOCK_PIN | MANUAL_UNLOCK_PIN);
    // Enable pull-ups for Launchpad switches
    GPIOPadConfigSet(MANUAL_CTRL_PORT, MANUAL_LOCK_PIN | MANUAL_UNLOCK_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Ignition Switch
    SysCtlPeripheralEnable(IGNITION_GPIO_PERIPH); // Might be same as GEAR/DOOR port
    while (!SysCtlPeripheralReady(IGNITION_GPIO_PERIPH));
    GPIOPinTypeGPIOInput(IGNITION_PORT, IGNITION_PIN);
    // Add pull-up/down if needed

    printf("DEBUG: Input Switches Initialized\n");
}


/* Initialize LCD Interface (4-bit mode example) */
void LCD_Init() {
    SysCtlPeripheralEnable(LCD_DATA_GPIO_PERIPH);
    SysCtlPeripheralEnable(LCD_CTRL_GPIO_PERIPH);
    while (!SysCtlPeripheralReady(LCD_DATA_GPIO_PERIPH));
    while (!SysCtlPeripheralReady(LCD_CTRL_GPIO_PERIPH));

    GPIOPinTypeGPIOOutput(LCD_DATA_PORT, LCD_D4_PIN | LCD_D5_PIN | LCD_D6_PIN | LCD_D7_PIN);
    GPIOPinTypeGPIOOutput(LCD_CTRL_PORT, LCD_RS_PIN | LCD_EN_PIN);

    // Initialization Sequence
    SysCtlDelay(SysCtlClockGet() / 3 / 50); // Wait >15ms
    LCD_Send_Nibble(0x3);
    SysCtlDelay(SysCtlClockGet() / 3 / 200); // Wait >4.1ms
    LCD_Send_Nibble(0x3);
    SysCtlDelay(SysCtlClockGet() / 3 / 10000); // Wait >100us
    LCD_Send_Nibble(0x3);
    SysCtlDelay(SysCtlClockGet() / 3 / 10000);
    LCD_Send_Nibble(0x2); // Set 4-bit mode
    SysCtlDelay(SysCtlClockGet() / 3 / 10000);

    LCD_Send_Cmd(0x28); // Function Set: 4-bit, 2-line, 5x8 font
    LCD_Send_Cmd(0x0C); // Display ON, Cursor OFF, Blink OFF
    LCD_Send_Cmd(0x06); // Entry Mode Set: Increment cursor, no shift
    LCD_Clear();        // Clear display
    printf("DEBUG: LCD Initialized\n");
}


// --- Peripheral Control Functions ---

/* Read Potentiometer ADC Value */
uint32_t Read_Potentiometer(void) {
    uint32_t adcValue[1];
    ADCProcessorTrigger(POT_ADC_BASE, POT_ADC_SEQUENCER);
    while (!ADCIntStatus(POT_ADC_BASE, POT_ADC_SEQUENCER, false));
    ADCIntClear(POT_ADC_BASE, POT_ADC_SEQUENCER);
    ADCSequenceDataGet(POT_ADC_BASE, POT_ADC_SEQUENCER, adcValue);
    return adcValue[0];
}

/* Trigger Ultrasonic Sensor Pulse */
void Trigger_Ultrasonic(void) {
    GPIOPinWrite(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN, ULTRA_TRIG_PIN); // HIGH
    // Accurate microsecond delay needed here. SysCtlDelay is approximate.
    // Use a dedicated timer or a calibrated loop for ~10-15us.
    SysCtlDelay(2); // VERY approximate, adjust based on clock speed, aim for >10us
    GPIOPinWrite(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN, 0);              // LOW
}

/* Read Ultrasonic Echo Duration using Timer */
uint32_t Read_Ultrasonic_Echo_Timer(void) {
    // WARNING: Polling implementation - Less accurate and blocks CPU.
    // An interrupt-driven approach using Timer Capture is strongly recommended.

    // Ensure Timer is enabled for capture
    TimerEnable(ULTRA_ECHO_TIMER_BASE, ULTRA_ECHO_TIMER);

    // 1. Wait for Echo Pin to go HIGH
    while (GPIOPinRead(ULTRA_ECHO_PORT, ULTRA_ECHO_PIN) == 0) {
        // Add timeout logic here to prevent getting stuck
    }

    // 2. Reset Timer value when echo starts
    HWREG(ULTRA_ECHO_TIMER_BASE + TIMER_O_TAV) = 0; // Reset Timer A value

    // 3. Wait for Echo Pin to go LOW
    while (GPIOPinRead(ULTRA_ECHO_PORT, ULTRA_ECHO_PIN) != 0) {
        // Add timeout logic here
    }

    // 4. Read Timer A value (captures duration)
    uint32_t timerVal = TimerValueGet(ULTRA_ECHO_TIMER_BASE, ULTRA_ECHO_TIMER);

    // Optional: Disable timer until next measurement
    // TimerDisable(ULTRA_ECHO_TIMER_BASE, ULTRA_ECHO_TIMER);

    return timerVal;
}

/* Calculate Speed from ADC Value */
int Calculate_Speed(uint32_t adcValue) {
    // Linear mapping: 0-4095 -> 0-100 km/h (adjust scale as needed)
    int speed = (int)(((float)adcValue / 4095.0f) * 100.0f);
    // Add clamping or non-linear mapping if necessary
    if (speed < 0) speed = 0;
    if (speed > 150) speed = 150; // Example max speed
    return speed;
}

/* Calculate Distance from Timer Value */
int Calculate_Distance(uint32_t timerValue) {
    // Distance (cm) = (Timer Value * Timer Period * Speed of Sound) / 2
    // Timer Period = Prescaler / System Clock Frequency
    // Speed of Sound ~ 34300 cm/s
    // Example: Clock=16MHz, Prescaler=16 -> Timer Tick = 16 / 16MHz = 1us = 1e-6 s
    // Distance = (timerValue * 1e-6 * 34300) / 2 = timerValue * 0.01715
    // Adjust the factor 0.01715 based on your actual clock and prescaler!
    float distance = (float)timerValue * 0.01715f;
    if (distance < 0) distance = 0;
    if (distance > 400) distance = 999; // Max practical range for HC-SR04, return indicator
    return (int)distance;
}

/* Activate Door Lock Mechanism */
void Lock_Doors() {
    GPIOPinWrite(LOCK_PORT, UNLOCK_PIN, 0); // Ensure unlock is off
    GPIOPinWrite(LOCK_PORT, LOCK_PIN, LOCK_PIN); // Activate lock
    // Optional: Add delay and turn off if pulse-driven
    printf("DEBUG: Locking Doors\n");
}

/* Deactivate Door Lock Mechanism */
void Unlock_Doors() {
    GPIOPinWrite(LOCK_PORT, LOCK_PIN, 0); // Ensure lock is off
    GPIOPinWrite(LOCK_PORT, UNLOCK_PIN, UNLOCK_PIN); // Activate unlock
    // Optional: Add delay and turn off if pulse-driven
    printf("DEBUG: Unlocking Doors\n");
}

/* Control RGB LED Colors */
void Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t pins_to_set = 0;
    uint32_t all_led_pins = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN;

    if (red > 0) pins_to_set |= LED_RED_PIN;
    if (green > 0) pins_to_set |= LED_GREEN_PIN;
    if (blue > 0) pins_to_set |= LED_BLUE_PIN;

    GPIOPinWrite(LED_PORT, all_led_pins, pins_to_set); // Write directly
}

/* Activate Buzzer */
void Activate_Buzzer(uint32_t freq) {
    // Simple ON/OFF implementation - Ignores frequency
    GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN); // Turn buzzer ON
}

/* Deactivate Buzzer */
void Deactivate_Buzzer() {
    GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0); // Turn buzzer OFF
}


// --- LCD Internal Helper Functions ---

static void LCD_Send_Nibble(uint8_t nibble) {
    uint32_t data_to_write = 0;
    uint32_t data_pins = LCD_D4_PIN | LCD_D5_PIN | LCD_D6_PIN | LCD_D7_PIN;

    if (nibble & 0x01) data_to_write |= LCD_D4_PIN;
    if (nibble & 0x02) data_to_write |= LCD_D5_PIN;
    if (nibble & 0x04) data_to_write |= LCD_D6_PIN;
    if (nibble & 0x08) data_to_write |= LCD_D7_PIN;

    GPIOPinWrite(LCD_DATA_PORT, data_pins, data_to_write);

    // Pulse EN
    GPIOPinWrite(LCD_CTRL_PORT, LCD_EN_PIN, LCD_EN_PIN);
    SysCtlDelay(10); // Short delay (adjust based on clock)
    GPIOPinWrite(LCD_CTRL_PORT, LCD_EN_PIN, 0);
    SysCtlDelay(10); // Short delay
}

static void LCD_Send_Cmd(uint8_t cmd) {
    GPIOPinWrite(LCD_CTRL_PORT, LCD_RS_PIN, 0); // RS Low for command
    LCD_Send_Nibble(cmd >> 4);   // Upper nibble
    LCD_Send_Nibble(cmd & 0x0F); // Lower nibble
    if (cmd == 0x01 || cmd == 0x02) { // Clear or Home commands take longer
        SysCtlDelay(SysCtlClockGet() / 3 / 500); // ~2ms delay
    } else {
        SysCtlDelay(SysCtlClockGet() / 3 / 20000); // ~50us delay
    }
}

static void LCD_Send_Data(uint8_t data) {
    GPIOPinWrite(LCD_CTRL_PORT, LCD_RS_PIN, LCD_RS_PIN); // RS High for data
    LCD_Send_Nibble(data >> 4);   // Upper nibble
    LCD_Send_Nibble(data & 0x0F); // Lower nibble
    SysCtlDelay(SysCtlClockGet() / 3 / 20000); // ~50us delay
}

static void LCD_Puts(const char *str) {
    while (*str) {
        LCD_Send_Data(*str++);
    }
}

static void LCD_GotoXY(uint8_t x, uint8_t y) {
    uint8_t address;
    switch (y) {
        case 0: address = 0x80; break; // Line 0 start
        case 1: address = 0xC0; break; // Line 1 start
        // Add cases for 20x4 etc. if needed
        default: address = 0x80; break;
    }
    address += x;
    LCD_Send_Cmd(address);
}

// --- Public LCD Functions ---

void LCD_Clear() {
    LCD_Send_Cmd(0x01); // Clear display command
}

void LCD_Display_Status(const char *status) {
    char buffer[17]; // 16 chars + null terminator
    snprintf(buffer, sizeof(buffer), "Doors: %-8s", status); // Format and pad
    LCD_GotoXY(0, 0);
    LCD_Puts(buffer);
}

void LCD_Display_Speed(int speed) {
    char buffer[9]; // "Spd:XXX " + null
    snprintf(buffer, sizeof(buffer), "Spd:%3d", speed);
    LCD_GotoXY(0, 1); // Start of line 1
    LCD_Puts(buffer);
}

void LCD_Display_Distance(int distance) {
    char buffer[9]; // " D:XXXcm" + null
    if (distance >= 0 && distance < 999) {
        snprintf(buffer, sizeof(buffer), " D:%3dcm", distance);
    } else {
        snprintf(buffer, sizeof(buffer), " D:--- "); // Indicate inactive
    }
    LCD_GotoXY(8, 1); // Approx middle of line 1 (for 16x2)
    LCD_Puts(buffer);
}

void LCD_Display_Message(const char *message) {
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "%-16s", message); // Pad/truncate to 16 chars
    LCD_GotoXY(0, 0); // Display general messages on line 0
    LCD_Puts(buffer);
}

