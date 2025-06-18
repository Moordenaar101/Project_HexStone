#include "Controller.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "utilities.h"
#include <Arduino.h>
#include <PS4Controller.h>

unsigned long lastTimeStamp = 0;

Vector2 getJoystickData(int joystickIndex) {
  Vector2 data;
  if (PS4.isConnected()) {
    if (joystickIndex == 0) {
      data.x = PS4.LStickX();
      data.y = PS4.LStickY();
    } else if (joystickIndex == 1) {
      data.x = PS4.RStickX();
      data.y = PS4.RStickY();
    } else {
      // Default to left joystick if index is invalid
      data.x = PS4.LStickX();
      data.y = PS4.LStickY();
    }
  } else {
    data.x = data.y = 0;
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
    Serial.printf("lx:%4d,ly:%4d,rx:%4d,ry:%4d\n", PS4.LStickX(), PS4.LStickY(),
                  PS4.RStickX(), PS4.RStickY());
#endif
#if SENSORS
    Serial.printf("gx:%5d,gy:%5d,gz:%5d,ax:%5d,ay:%5d,az:%5d\n", PS4.GyrX(),
                  PS4.GyrY(), PS4.GyrZ(), PS4.AccX(), PS4.AccY(), PS4.AccZ());
#endif
    lastTimeStamp = millis();
  }
}

void onDisConnect() { Serial.println("Disconnected!"); }