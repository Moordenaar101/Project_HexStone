#include "LegUtilities.h"
#include "Utilities.h"
#include "controller.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

// Extern variables from main.cpp
extern int coxaLen;
extern int femurLen;
extern int tibiaLen;
extern int bodyHeight;
extern bool debug;

// Internal Variables
const int cycleResolution = 1000;
int legStates[6];
int cycleProgress[6];
Vector3 cycleStartPoints[6];
float tArray[6];
Vector2 joy1TargetVect;
float joy1TargetMagnitude;
Vector2 joy2TargetVect;
float joy2TargetMagnitude;

Vector2 joy1CurrentVect;
float joy1CurrentMagnitude;
Vector2 joy2CurrentVect;
float joy2CurrentMagnitude;

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
      sqrt(pow(adjustedDistVect.x, 2) + pow(adjustedDistVect.y, 2));
  const double hipToGoal = sqrt(pow(h, 2) + pow(adjustedDistVect.z, 2));

  const double coxaAngle =
      atan2(adjustedDistVect.y, adjustedDistVect.x); // Coxa

  const double angleA = atan((adjustedDistVect.z / h));
  const double angleB =
      acos((pow(hipToGoal, 2) + pow(femurLen, 2) - pow(tibiaLen, 2)) /
           (2 * hipToGoal * femurLen));

  const double femurAngle = angleB - angleA; // Femur
  const double tibiaAngle =
      acos((pow(femurLen, 2) + pow(tibiaLen, 2) - pow(hipToGoal, 2)) /
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

void setServoPositions(Legtype leg, Vector3 angles,
                       Adafruit_PWMServoDriver &controller) {
  // This function sets the servo positions for the given leg object

  if (leg.legNumber <= 4) {
    controller.setPWM(leg.legNumber * 3, 0,
                      map(angles.x, 0, 180, SERVOMIN, SERVOMAX)); // Coxa
    controller.setPWM(leg.legNumber * 3 + 1, 0,
                      map(angles.y, 0, 180, SERVOMIN, SERVOMAX)); // Femur
    controller.setPWM(leg.legNumber * 3 + 2, 0,
                      map(angles.z, 0, 180, SERVOMIN, SERVOMAX)); // Tibia
  } else {
    // For the last two servos, use analogWrite
    // This is a workaround for the PCA9685 only having 16 channels
    // *** This will need to be revisited after the IK has been fully
    // implemented! ***
    controller.setPWM(leg.legNumber * 3, 0,
                      map(angles.x, 0, 180, SERVOMIN, SERVOMAX)); // Coxa
    analogWrite(SERVOPIN_16, map(angles.x, 0, 180, 0, 255));      // Femur
    analogWrite(SERVOPIN_17, map(angles.x, 0, 180, 0, 255));      // Tibia
  }
}

void initWalkMode(const Gait &gait, Legtype leg) {
  for (int i = 0; i < 6; i++) {
    legStates[i] = Reset; // Reset all leg states
    cycleProgress[i] = gait.offsets[i] * cycleResolution;
  }
};

void standMode(void) {}

void walkMode(const Gait &gait, Legtype leg) {
  joy1TargetVect = {(float)map(getJoystickData(0).x, -127, 127, -100, 100),
                    (float)map(getJoystickData(0).y, -127, 127, -100, 100)};

  joy1TargetMagnitude =
      constrain(hypot(joy1TargetVect.x, joy1TargetVect.y), 0, 100);

  joy2TargetVect = {(float)map(getJoystickData(1).x, -127, 127, -100, 100),
                    (float)map(getJoystickData(1).y, -127, 127, -100, 100)};

  joy2TargetMagnitude =
      constrain(hypot(joy2TargetVect.x, joy2TargetVect.y), 0, 100);

  joy1CurrentVect.lerp(joy1TargetVect, 0.04);
  joy1CurrentMagnitude = lerp(joy1CurrentMagnitude, joy1TargetMagnitude, 0.04);

  joy2CurrentVect.lerp(joy2TargetVect, 0.06);
  joy2CurrentMagnitude = lerp(joy2CurrentMagnitude, joy2TargetMagnitude, 0.06);

  for (int i = 0; i < 6; i++) {
    tArray[i] = (float)cycleProgress[i] / cycleResolution;
  }

  // Vector3 getGaitCycle(gait, leg){}
}

Vector3 getGaitPoint(const Gait &gait, Legtype leg, float t) {

  if (t < gait.cycleRatio) {
    if (legStates[leg.legNumber] != Propelling)
      cycleStartPoints[leg.legNumber] = leg.footPosition;
  }
}