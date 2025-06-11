#include "LegUtilities.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

Adafruit_PWMServoDriver pcaDriver = Adafruit_PWMServoDriver();

// Extern variables from main.cpp
extern int coxaLen;
extern int femurLen;
extern int tibiaLen;
extern int bodyHeight;
extern bool debug;

Vector3 inverseKinematics(Legtype leg, const Vector3 &goal) {
  // Returns a vector3 of angles given a leg object and a goal vector
  // X = coxa angle, Y = femur angle, Z = tibia angle

  const double xDistance = goal.x - leg.legOrigin.x;
  const double yDistance = goal.y - leg.legOrigin.y;

  const double theta = atan2(yDistance, xDistance);

  Vector3 effectiveGoal(goal.x - coxaLen * cos(theta),
                        goal.y - coxaLen * sin(theta), goal.z);

  // Calculate the distance from the femur servo to the effective goal
  const double hipToGoalDistance = effectiveGoal.distanceTo(
      Vector3(leg.legOrigin.x, leg.legOrigin.y, bodyHeight));

  // Adjust goal if out of bounds
  if (hipToGoalDistance > femurLen + tibiaLen)
    effectiveGoal.lerp(Vector3(leg.legOrigin.x, leg.legOrigin.y, bodyHeight),
                       1 - (femurLen + tibiaLen - 0.001f) / hipToGoalDistance);

  //  accounting for the distance between the coxa and femur servos
  const Vector3 adjustedDistVect(effectiveGoal.x - leg.legOrigin.x,
                                 effectiveGoal.y - leg.legOrigin.y,
                                 bodyHeight - effectiveGoal.z);

  const double h =
      sqrt(square(adjustedDistVect.x) + square(adjustedDistVect.y));
  const double hipToGoal = sqrt(square(h) + square(adjustedDistVect.z));

  const double coxaAngle =
      atan2(adjustedDistVect.y, adjustedDistVect.x); // Coxa

  const double angleA = atan((adjustedDistVect.z / h));
  const double angleB =
      acos((square(hipToGoal) + square(femurLen) - square(tibiaLen)) /
           (2 * hipToGoal * femurLen));

  const double femurAngle = angleB - angleA; // Femur
  const double tibiaAngle =
      acos((square(femurLen) + square(tibiaLen) - square(hipToGoal)) /
           (2.0f * femurLen * tibiaLen)); // Tibia

  Vector3 angles(coxaAngle, femurAngle, tibiaAngle);

  if (debug) {
    Serial.println("\n\n\nGoal position: " + goal.toString());
    Serial.println("Leg Origin: " + String(leg.legOrigin.x) + ", " +
                   String(leg.legOrigin.y) + ", " + String(bodyHeight));
    Serial.println("Raw Angles (Rad) : " + String(angles.x) + ", " +
                   String(angles.y) + ", " + String(angles.z));
    Serial.println("Raw Angles (Deg) : " + String(radToDeg(angles.x)) + ", " +
                   String(radToDeg(angles.y), 5) + ", " +
                   String(radToDeg(angles.z)));
    Serial.println(
        "PWM Signal: " +
        String(map(angles.x, -M_PI, M_PI, SERVOMIN, SERVOMAX)) + ", " +
        String(map(angles.y, -M_PI, M_PI, SERVOMIN, SERVOMAX)) + ", " +
        String(map(angles.z, -M_PI, M_PI, SERVOMIN, SERVOMAX)));
    Serial.println("Other values:\nAngle A: " + String(radToDeg(angleA)) +
                   "\nAngle B: " + String(radToDeg(angleB)) +
                   "\nH: " + String(h) + "\nTheta: " + String(radToDeg(theta)) +
                   "\nAdjusted Dist Vect: " + adjustedDistVect.toString() +
                   "\nReachable Goal: " + effectiveGoal.toString() +
                   "\nHip To Goal: " + String(hipToGoal));
  }

  return angles; // Return the angles in radians
}

void setServoPositions(Legtype leg, Vector3 angles) {
  // This function sets the servo positions for the given leg object

  if (leg.legNumber <= 4) {
    pcaDriver.setPWM(leg.legNumber * 3, 0,
                     map(angles.x, 0, 180, SERVOMIN, SERVOMAX)); // Coxa
    pcaDriver.setPWM(leg.legNumber * 3 + 1, 0,
                     map(angles.y, 0, 180, SERVOMIN, SERVOMAX)); // Femur
    pcaDriver.setPWM(leg.legNumber * 3 + 2, 0,
                     map(angles.z, 0, 180, SERVOMIN, SERVOMAX)); // Tibia
  } else {
    // For the last two servos, use analogWrite
    // This is a workaround for the PCA9685 only having 16 channels
    // *** This will need to be revisited after the IK has been fully
    // implemented! ***
    pcaDriver.setPWM(leg.legNumber * 3, 0,
                     map(angles.x, 0, 180, SERVOMIN, SERVOMAX)); // Coxa
    analogWrite(SERVOPIN_16, map(angles.x, 0, 180, 0, 255));     // Femur
    analogWrite(SERVOPIN_17, map(angles.x, 0, 180, 0, 255));     // Tibia
  }
}
