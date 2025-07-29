#include "LegUtilities.h"
#include "Utilities.h"
#include "controller.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

// Extern variables from main.cpp
extern float coxaLen;
extern float femurLen;
extern float tibiaLen;
extern float bodyHeight;
extern bool debug;

// Internal Variables
// const int cycleResolution = 1000;
// int legStates[6];
// int cycleProgress[6];
// Vector3 cycleStartPoints[6];
// float tArray[6];
// float frameHeight = 45;
// const float globalRotationFactor = 0.1;
// const float globalStrideFactor = 0.1;
// const int centerDist =
//     100; // Distance from the center of the hexapod to the coxa of each leg
// const float strideMultiplier[6] = {-1, -1, -1, 1, 1, 1};
// const float rotationMultiplier[6] = {1, 0, -1, 1, 0, -1};
// const float legLandHeight = 25;
// const float legPlacementAngle = 55;

Vector3 inverseKinematics(Vector2 legOrigin, const Vector3 &goal) {

  /***** Re-writing IK due to calculation errors *****/
  float x = goal.x - legOrigin.x;
  float y = goal.y - legOrigin.y;
  float z = -bodyHeight + goal.z;

  float theta1 = atan2(y, x); // Tibia
  float l = sqrt(sq(x) + sq(y)) - coxaLen * (legOrigin.x / abs(legOrigin.x));
  float h = sqrt(sq(l) + sq(z));

  float phi1 = acos(constrain(
      (sq(h) + sq(femurLen) - sq(tibiaLen)) / (2 * h * femurLen), -1, 1));
  float phi2 = atan2(z, l);
  float theta2 = (phi1 + phi2); // Femur
  float phi3 = acos(constrain((sq(femurLen) + sq(tibiaLen) - sq(h)) /
                                  (2 * femurLen * tibiaLen),
                              -1, 1));
  float theta3 = phi3; // Coxa

  //   Serial.println(
  //       "New IK Results: " +
  //       Vector3(degrees(theta1), degrees(theta2),
  //       degrees(theta3)).toString());

  Vector3 angles;

  angles.x = theta1;        // Coxa
  angles.y = theta2;        // Femur
  angles.z = M_PI - theta3; // Tibia

  return angles; // Return the angles in radians
}
