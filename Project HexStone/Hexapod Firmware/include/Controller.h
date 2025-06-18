#pragma once

#include "Utilities.h"
#include <Arduino.h>
#include <PS4Controller.h>

// Macros used in the .cpp
#define EVENTS 0
#define BUTTONS 0
#define JOYSTICKS 1
#define SENSORS 0

// Extern for lastTimeStamp variable
extern unsigned long lastTimeStamp;

Vector2 getJoystickData(int joystickIndex);
void removePairedDevices();
void printDeviceAddress();
void onConnect();
void notify();
void onDisConnect();
