#include "Controller.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "utilities.h"
#include <Arduino.h>
#include <PS4Controller.h>

unsigned long lastTimeStamp = 0;

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

void onConnect() { Serial.println("Connected!"); }

void notify() {
#if EVENTS
  boolean sqd = PS4.event.button_down.square, squ = PS4.event.button_up.square,
          trd = PS4.event.button_down.triangle,
          tru = PS4.event.button_up.triangle;
  if (sqd)
    Serial.println("SQUARE down");
  else if (squ)
    Serial.println("SQUARE up");
  else if (trd)
    Serial.println("TRIANGLE down");
  else if (tru)
    Serial.println("TRIANGLE up");
#endif

#if BUTTONS
  boolean sq = PS4.Square(), tr = PS4.Triangle();
  if (sq)
    Serial.print(" SQUARE pressed");
  if (tr)
    Serial.print(" TRIANGLE pressed");
  if (sq | tr)
    Serial.println();
#endif

  // Only needed to print the message properly on serial monitor. Else we dont
  // need it.
  if (millis() - lastTimeStamp > 50) {
#if JOYSTICKS
    Serial.printf("lx:%4d, ly:%4d, rx:%4d, ry:%4d\n", PS4.LStickX(),
                  PS4.LStickY(), PS4.RStickX(), PS4.RStickY());
#endif
#if SENSORS
    Serial.printf("gx:%5d,gy:%5d,gz:%5d,ax:%5d,ay:%5d,az:%5d\n", PS4.GyrX(),
                  PS4.GyrY(), PS4.GyrZ(), PS4.AccX(), PS4.AccY(), PS4.AccZ());
#endif
    lastTimeStamp = millis();
  }
}

void onDisConnect() { Serial.println("Disconnected!"); }