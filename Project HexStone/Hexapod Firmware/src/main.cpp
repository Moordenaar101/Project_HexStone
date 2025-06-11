#include "LegUtilities.h"
#include "Utilities.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/*** TEMPERARY VARIABLE DECLARATION ***/
int coxaLen = 45.0f;    // Length of the coxa segment
int femurLen = 100.0f;  // Length of the femur segment
int tibiaLen = 180.0f;  // Length of the tibia segment
int bodyHeight = 50.0f; // Height of the chassis, used for gait calculations
Legtype legs[NUM_LEGS]; // Placeholder for Servo Data
String servoNames[3] = {"Coxa", "Femur", "Tibia"}; // Names for each servo
// const Vector2 legOrigins[6] = { // For testing!
//     Vector2(-70, 85), Vector2(-70, 0), Vector2(-70, -85),
//     Vector2(70, -85), Vector2(70, 0),  Vector2(70, 85)}; // Leg origins
// const Vector2 gaitOrigins[6] = {
//     Vector2(-150, 200), Vector2(-170, 0), Vector2(-150, -200),
//     Vector2(150, -200), Vector2(170, 0),  Vector2(150, 200)}; // Gait origins
const Vector2 legOrigins[6] = {Vector2(0, 0)};
const Vector2 gaitOrigins[6] = {Vector2(150, 0)};
bool debug = true; // Debug flag
/**************************************/

// Hexapod Layout
/*
          Y
          ↑
          + → X

        Front
   (0)         (5)
     \  __↑__  /
      \/     \/
(1)___|       |___(4)
      |       |
      /\_____/\
     /         \
   (2)         (3)
        Back

 Parts:
 0 = Coxa
 1 = Femur
 2 = Tibia

          2
        // \\
 |0 == 1    \\
             \\
*/

Vector3 inverseKinematics(Legtype leg, const Vector3 &goal);
void setServoPositions(Legtype leg, Vector3 angles);

void setup() {
  Serial.begin(9600);
  Serial.println("Single Leg Test");
  // pcaDriver.begin();
  // pcaDriver.setOscillatorFrequency(26000000);
  // pcaDriver.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  pinMode(SERVOPIN_16, OUTPUT); // Set the pin modes for the servos
  pinMode(SERVOPIN_17, OUTPUT);

  for (int i = 0; i < NUM_LEGS; i++) { // Construct the leg objects
    legs[i].legNumber = i;
    legs[i].gaitOrigin = gaitOrigins[i];
    legs[i].legOrigin = legOrigins[i];
    legs[i].footPosition = gaitOrigins[i];
  }

  delay(10);

  setServoPositions(legs[0], inverseKinematics(legs[0], Vector3(225, 45, 5)));
}

void loop() {}
