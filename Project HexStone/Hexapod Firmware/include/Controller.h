#pragma once

#include "Utilities.h"
#include <Arduino.h>
#include <PS4Controller.h>
#include <ps5Controller.h>

// Macros used in the .cpp
#define EVENTS 0
#define BUTTONS 0
#define JOYSTICKS 0
#define SENSORS 0

// Data type to hold all controller inputs
struct ControllerData {
  Vector2 leftStick;
  Vector2 rightStick;
  bool buttonCross;
  bool buttonCircle;
  bool buttonSquare;
  bool buttonTriangle;
  bool buttonL1;
  bool buttonR1;
  bool buttonL2;
  bool buttonR2;
  bool buttonShare;
  bool buttonOptions;
  bool buttonUp;
  bool buttonDown;
  bool buttonLeft;
  bool buttonRight;
  bool buttonTouchpad;
  int16_t l2Value;
  int16_t r2Value;
  // Add more fields as needed
};

// Extern for lastTimeStamp variable
extern unsigned long lastTimeStamp;

ControllerData getJoystickData();
// void removePairedDevices();
// void printDeviceAddress();
// void onConnect();
// void notify();
// void onDisConnect();
bool inputDetected();