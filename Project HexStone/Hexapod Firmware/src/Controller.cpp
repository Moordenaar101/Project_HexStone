#include "Controller.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "utilities.h"
#include <Arduino.h>
#include <PS4Controller.h>

// Deadzone for joystick input
int deadZone = 10;

ControllerData getJoystickData() {
  ControllerData data;
  data.leftStick = Vector2(0, 0);
  data.rightStick = Vector2(0, 0);
  data.buttonCross = false;
  data.buttonCircle = false;
  data.buttonSquare = false;
  data.buttonTriangle = false;
  data.buttonL1 = false;
  data.buttonR1 = false;
  data.buttonL2 = false;
  data.buttonR2 = false;
  data.buttonShare = false;
  data.buttonOptions = false;
  data.buttonUp = false;
  data.buttonDown = false;
  data.buttonLeft = false;
  data.buttonRight = false;
  data.buttonTouchpad = false;
  data.l2Value = 0;
  data.r2Value = 0;

  if (PS4.isConnected()) {
    data.leftStick.x = PS4.LStickX();
    data.leftStick.y = PS4.LStickY();
    data.rightStick.x = PS4.RStickX();
    data.rightStick.y = PS4.RStickY();
    data.buttonCross = PS4.Cross();
    data.buttonCircle = PS4.Circle();
    data.buttonSquare = PS4.Square();
    data.buttonTriangle = PS4.Triangle();
    data.buttonL1 = PS4.L1();
    data.buttonR1 = PS4.R1();
    data.buttonL2 = PS4.L2();
    data.buttonR2 = PS4.R2();
    data.buttonShare = PS4.Share();
    data.buttonOptions = PS4.Options();
    data.buttonUp = PS4.Up();
    data.buttonDown = PS4.Down();
    data.buttonLeft = PS4.Left();
    data.buttonRight = PS4.Right();
    data.buttonTouchpad = PS4.Touchpad();
    data.l2Value = PS4.L2Value();
    data.r2Value = PS4.R2Value();
    // Add more fields as needed
  }
  return data;
}

void removePairedDevices() {
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++) {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void printDeviceAddress() {
  const uint8_t *point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02x", (int)point[i]);
    Serial.print(str);
    if (i < 5) {
      Serial.print(":");
    }
  }
}

void onConnect() { Serial.println("Controller Connected!"); }

void onDisConnect() { Serial.println("Controller Disconnected!"); }

bool inputDetected() {
  bool input = false;
  ControllerData data = getJoystickData();

  if (abs(data.leftStick.x) > deadZone || abs(data.leftStick.y) > deadZone ||
      abs(data.rightStick.y) > deadZone) {
    input = true;
  }
  return input;
}