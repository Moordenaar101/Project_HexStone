#include "Controller.h"
// #include "esp_bt_device.h"
// #include "esp_bt_main.h"
// #include "esp_err.h"
// #include "esp_gap_bt_api.h"
#include "utilities.h"
#include <Arduino.h>
#include <ps5Controller.h>

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

  if (ps5.isConnected()) {
    data.leftStick.x = ps5.LStickX();
    data.leftStick.y = ps5.LStickY();
    data.rightStick.x = ps5.RStickX();
    data.rightStick.y = ps5.RStickY();
    data.buttonCross = ps5.Cross();
    data.buttonCircle = ps5.Circle();
    data.buttonSquare = ps5.Square();
    data.buttonTriangle = ps5.Triangle();
    data.buttonL1 = ps5.L1();
    data.buttonR1 = ps5.R1();
    data.buttonL2 = ps5.L2();
    data.buttonR2 = ps5.R2();
    data.buttonShare = ps5.Share();
    data.buttonOptions = ps5.Options();
    data.buttonUp = ps5.Up();
    data.buttonDown = ps5.Down();
    data.buttonLeft = ps5.Left();
    data.buttonRight = ps5.Right();
    data.buttonTouchpad = ps5.Touchpad();
    data.l2Value = ps5.L2Value();
    data.r2Value = ps5.R2Value();
    // Add more fields as needed
  }
  return data;
}

// void removePairedDevices() {
//   uint8_t pairedDeviceBtAddr[20][6];
//   int count = esp_bt_gap_get_bond_device_num();
//   esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
//   for (int i = 0; i < count; i++) {
//     esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
//   }
// }

// void printDeviceAddress() {
//   const uint8_t *point = esp_bt_dev_get_address();
//   for (int i = 0; i < 6; i++) {
//     char str[3];
//     sprintf(str, "%02x", (int)point[i]);
//     Serial.print(str);
//     if (i < 5) {
//       Serial.print(":");
//     }
//   }
// }

// void onConnect() { Serial.println("Controller Connected!"); }

// void onDisConnect() { Serial.println("Controller Disconnected!"); }

bool inputDetected() {
  bool input = false;
  ControllerData data = getJoystickData();

  if (abs(data.leftStick.x) > deadZone || abs(data.leftStick.y) > deadZone ||
      abs(data.rightStick.y) > deadZone) {
    input = true;
  }
  return input;
}