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

Vector3 inverseKinematics(Vector2 legOrigin, float oldCoxa,
                          const Vector3 &goal) {
  // Returns a vector3 of angles given a leg object and a goal vector
  // X = coxa angle, Y = femur angle, Z = tibia angle

  Serial.println("\n");
  Serial.println(goal.toString());

  //   const double xDistance = goal.x - legOrigin.x;
  //   const double yDistance = goal.y - legOrigin.y;

  //   const double theta = atan2(yDistance, xDistance);

  //   Vector3 effectiveGoal(goal.x - coxaLen * cos(theta),
  //                         goal.y - coxaLen * sin(theta), goal.z);

  //   // Calculate the distance from the femur servo to the effective goal
  //   const double hipToGoalDistance =
  //       effectiveGoal.distanceTo(Vector3(legOrigin.x, legOrigin.y,
  //       bodyHeight));

  //   // Adjust goal if out of bounds
  //   if (hipToGoalDistance > femurLen + tibiaLen) {
  //     const Vector3 oldGoal = effectiveGoal;
  //     effectiveGoal = effectiveGoal.lerp(
  //         Vector3(legOrigin.x, legOrigin.y, bodyHeight),
  //         1 - (femurLen + tibiaLen - 0.001f) / hipToGoalDistance);

  //     Serial.println(
  //         "WARNING: Goal out of reach! Adjusted to: " +
  //         effectiveGoal.toString() +
  //         "\nThe original goal was: " + oldGoal.toString());
  //   }

  //   //  accounting for the distance between the coxa and femur servos
  //   const Vector3 adjustedDistVect(effectiveGoal.x - legOrigin.x,
  //                                  effectiveGoal.y - legOrigin.y,
  //                                  bodyHeight - effectiveGoal.z);

  //   const double h =
  //       sqrt(pow(adjustedDistVect.x, 2) + pow(adjustedDistVect.y, 2));
  //   const double hipToGoal = sqrt(pow(h, 2) + pow(adjustedDistVect.z, 2));

  //   const double coxaAngle =
  //       atan2(adjustedDistVect.y, adjustedDistVect.x); // Coxa

  //   const double angleA = atan((adjustedDistVect.z / h));
  //   const double angleB =
  //       acos((pow(hipToGoal, 2) + pow(femurLen, 2) - pow(tibiaLen, 2)) /
  //            (2 * hipToGoal * femurLen));

  //   const double femurAngle = angleB - angleA; // Femur
  //   const double tibiaAngle =
  //       acos((pow(femurLen, 2) + pow(tibiaLen, 2) - pow(hipToGoal, 2)) /
  //            (2.0f * femurLen * tibiaLen)); // Tibia

  //   Serial.println("IK Results: " + Vector3(degrees(coxaAngle),
  //                                           degrees(femurAngle),
  //                                           degrees(tibiaAngle))
  //                                       .toString());

  //   Vector3 angles(coxaAngle, femurAngle, tibiaAngle);

  //   if (debug) {
  //     Serial.println("\n\n\nGoal position: " + goal.toString());
  //     Serial.println("Leg Origin: " + String(legOrigin.x) + ", " +
  //                    String(legOrigin.y) + ", " + String(bodyHeight));
  //     Serial.println("Raw Angles (Rad) : " + String(angles.x) + ", " +
  //                    String(angles.y) + ", " + String(angles.z));
  //     Serial.println("Raw Angles (Deg) : " + String(radToDeg(angles.x)) + ",
  //     " +
  //                    String(radToDeg(angles.y), 5) + ", " +
  //                    String(radToDeg(angles.z)));
  //     // Serial.println(
  //     //     "PWM Signal: " +
  //     //     String(map(angles.x, -M_PI, M_PI, SERVOMIN, SERVOMAX)) + ", " +
  //     //     String(map(angles.y, -M_PI, M_PI, SERVOMIN, SERVOMAX)) + ", " +
  //     //     String(map(angles.z, -M_PI, M_PI, SERVOMIN, SERVOMAX)));
  //     Serial.println("Other values:\nAngle A: " + String(radToDeg(angleA)) +
  //                    "\nAngle B: " + String(radToDeg(angleB)) +
  //                    "\nH: " + String(h) + "\nTheta: " +
  //                    String(radToDeg(theta)) +
  //                    "\nAdjusted Dist Vect: " + adjustedDistVect.toString() +
  //                    "\nReachable Goal: " + effectiveGoal.toString() +
  //                    "\nHip To Goal: " + String(hipToGoal));

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
//       Vector3(degrees(theta1), degrees(theta2), degrees(theta3)).toString());

  Vector3 angles;

  angles.x = theta1; // Coxa
  angles.y = theta2; // Femur
  angles.z = theta3; // Tibia

  return angles; // Return the angles in radians
}
