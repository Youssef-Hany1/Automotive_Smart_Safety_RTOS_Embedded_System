#ifndef DRIVERS_H
#define DRIVERS_H

void initGPIO();
void initADC();
void initUltrasonic();
void initBuzzer();
void initRGB();
int readSpeedADC();
int measureDistance();
void setBuzzerFrequency(int frequency);
void setRGBColor(char color);
int isGearDrive();
int isGearReverse();
int isIgnitionOn();
int isDriverDoorOpen();
void lockDoors();
void unlockDoors();
int isManualLockPressed();
int isManualUnlockPressed();

#endif