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
float frameHeight = 45;
const float globalRotationFactor = 0.1;
const float globalStrideFactor = 0.1;
const int centerDist =
    100; // Distance from the center of the hexapod to the coxa of each leg
const float strideMultiplier[6] = {-1, -1, -1, 1, 1, 1};
const float rotationMultiplier[6] = {1, 0, -1, 1, 0, -1};
const float legLandHeight = 25;
const float legPlacementAngle = 55;

Vector3 inverseKinematics(Vector2 legOrigin, const Vector3 &goal) {
  // Returns a vector3 of angles given a leg object and a goal vector
  // X = coxa angle, Y = femur angle, Z = tibia angle

  const double xDistance = goal.x - legOrigin.x;
  const double yDistance = goal.y - legOrigin.y;

  const double theta = atan2(yDistance, xDistance);

  Vector3 effectiveGoal(goal.x - coxaLen * cos(theta),
                        goal.y - coxaLen * sin(theta), goal.z);

  // Calculate the distance from the femur servo to the effective goal
  const double hipToGoalDistance = effectiveGoal.distanceTo(
      Vector3(legOrigin.x, legOrigin.y, bodyHeight));

  // Adjust goal if out of bounds
  if (hipToGoalDistance > femurLen + tibiaLen)
    effectiveGoal.lerp(Vector3(legOrigin.x, legOrigin.y, bodyHeight),
                       1 - (femurLen + tibiaLen - 0.001f) / hipToGoalDistance);

  //  accounting for the distance between the coxa and femur servos
  const Vector3 adjustedDistVect(effectiveGoal.x - legOrigin.x,
                                 effectiveGoal.y - legOrigin.y,
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
    Serial.println("Leg Origin: " + String(legOrigin.x) + ", " +
                   String(legOrigin.y) + ", " + String(bodyHeight));
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

void gaitMode::init(const Gait &gait) {
  for (int i = 0; i < 6; i++) {
    legStates[i] = Reset; // Reset all leg states
    cycleProgress[i] = gait.offsets[i] * cycleResolution;
  }
}

void gaitMode::stand(const Gait &gait, Legtype leg) {}

void gaitMode::loop(const Gait &gait, Legtype leg) {
  joy1TargetVect = {
      (float)map(getJoystickData().leftStick.x, -127, 127, -100, 100),
      (float)map(getJoystickData().leftStick.y, -127, 127, -100, 100)};

  joy1TargetMagnitude =
      constrain(hypot(joy1TargetVect.x, joy1TargetVect.y), 0, 100);

  joy2TargetVect = {
      (float)map(getJoystickData().rightStick.x, -127, 127, -100, 100),
      (float)map(getJoystickData().rightStick.y, -127, 127, -100, 100)};

  joy2TargetMagnitude =
      constrain(hypot(joy2TargetVect.x, joy2TargetVect.y), 0, 100);

  joy1CurrentVect.lerp(joy1TargetVect, 0.04);
  joy1CurrentMagnitude = lerp(joy1CurrentMagnitude, joy1TargetMagnitude, 0.04);

  joy2CurrentVect.lerp(joy2TargetVect, 0.06);
  joy2CurrentMagnitude = lerp(joy2CurrentMagnitude, joy2TargetMagnitude, 0.06);

  for (int i = 0; i < 6; i++) {
    tArray[i] = (float)cycleProgress[i] / cycleResolution;
  };
}

void gaitMode::exit() {
  // Add any cleanup logic if needed
}

Vector3 gaitMode::getGaitCycle(const Gait &gait, Legtype leg) {

  float rotationAmount = joy2CurrentVect.x * globalRotationFactor;

  Vector2 strafeStrideLength = joy1CurrentVect * gait.strideLengthFactor;
  strafeStrideLength.y =
      constrain(strafeStrideLength.y, -gait.maxStrideLength / 2,
                gait.maxStrideLength / 2);
  strafeStrideLength.x = constrain(strafeStrideLength.x, -gait.maxStrideLength,
                                   gait.maxStrideLength);

  float t = tArray[leg.legNumber];

  if (t < gait.cycleRatio) { // Pushing phase
    if (legStates[leg.legNumber] != Propelling)
      cycleStartPoints[leg.legNumber] = leg.footPosition;
    legStates[leg.legNumber] = Propelling;

    //-----This cycle is a straight line that will cause the hexapod to
    // strafe-----//

    // Starting point of the line
    vector<Vector3> strafeControlPoints = vector<Vector3>(2);
    strafeControlPoints[0] = cycleStartPoints[leg.legNumber];

    // Ending point of the line
    strafeControlPoints[1] =
        Vector3(strafeStrideLength.y * strideMultiplier[leg.legNumber], // X
                -strafeStrideLength.x * strideMultiplier[leg.legNumber] +
                    centerDist, // Y
                frameHeight     // Z
                )
            .rotate(legPlacementAngle * rotationMultiplier[leg.legNumber],
                    Vector2(0, centerDist));

    Vector3 strafePoint = GetPointOnBezierCurve(
        strafeControlPoints, fastMap(t, 0, gait.cycleRatio, 0, 1));
    //-------------------------------------------------------------------------------//

    // Starting point of the curve
    vector<Vector3> rotateControlPoints = vector<Vector3>(3);
    rotateControlPoints[0] = cycleStartPoints[leg.legNumber];

    // Middle point of the curve
    rotateControlPoints[1] = Vector3(0,          // X
                                     centerDist, // Y
                                     frameHeight // Z
    );

    // Ending point of the curve
    rotateControlPoints[2] = Vector3(rotationAmount, // X
                                     centerDist,     // Y
                                     frameHeight     // Z
    );

    Vector3 rotatePoint = GetPointOnBezierCurve(
        rotateControlPoints, fastMap(t, 0, gait.cycleRatio, 0, 1));
    //-------------------------------------------------------------------------------//

    // Return the weighted average of the two points
    return (strafePoint * abs(joy1CurrentMagnitude) +
            rotatePoint * abs(joy2CurrentVect.x)) /
           (abs(joy1CurrentMagnitude) + abs(joy2CurrentVect.x));
  }

  // Lifting
  else {
    if (legStates[leg.legNumber] != Lifting)
      cycleStartPoints[leg.legNumber] = leg.footPosition;
    legStates[leg.legNumber] = Lifting;

    //------This cycle will cause the hexapod leg to lift up and return to the
    // beginning of the walk cycle in a straight line-----//

    // Starting point of the curve
    vector<Vector3> strafeControlPoints = vector<Vector3>(4);
    strafeControlPoints[0] = cycleStartPoints[leg.legNumber];

    // Control point directly above the starting point causing the leg to lift
    // up quickly
    strafeControlPoints[1] =
        cycleStartPoints[leg.legNumber] +
        Vector3(0, 0, gait.liftHeight * globalStrideFactor);

    // Control point directly above the ending point preventing the leg from
    // running into the ground
    strafeControlPoints[2] =
        Vector3(-strafeStrideLength.y * strideMultiplier[leg.legNumber], // X
                strafeStrideLength.x * strideMultiplier[leg.legNumber] +
                    centerDist,             // Y
                frameHeight + legLandHeight // Z
                )
            .rotate(legPlacementAngle * rotationMultiplier[leg.legNumber],
                    Vector2(0, centerDist));

    // Ending point of the curve
    strafeControlPoints[3] =
        Vector3(-strafeStrideLength.y * strideMultiplier[leg.legNumber],
                strafeStrideLength.x * strideMultiplier[leg.legNumber] +
                    centerDist,
                frameHeight)
            .rotate(legPlacementAngle * rotationMultiplier[leg.legNumber],
                    Vector2(0, centerDist));

    Vector3 straightPoint = GetPointOnBezierCurve(
        strafeControlPoints, fastMap(t, gait.cycleRatio, 1, 0, 1));
    //-------------------------------------------------------------------------------//

    //------This cycle will cause the hexapod leg to lift up and return to the
    // beginning of the walk cycle in a curved line-----//

    // Starting point of the curve
    vector<Vector3> rotateControlPoints = vector<Vector3>(5);
    rotateControlPoints[0] = cycleStartPoints[leg.legNumber];

    // Control point directly above the starting point causing the leg to lift
    // up quickly
    rotateControlPoints[1] =
        cycleStartPoints[leg.legNumber] +
        Vector3(0, 0, gait.liftHeight * globalStrideFactor);

    // Control point at the apex of the curve and offset away from the hexapods
    // body, cause the leg to lift up and away.
    rotateControlPoints[2] =
        Vector3(0,                                                 // X
                centerDist,                                        // Y
                frameHeight + gait.liftHeight * globalStrideFactor // Z
        );

    // Control point directly above the ending point preventing the leg from
    // running into the ground
    rotateControlPoints[3] = Vector3(-joy1CurrentMagnitude,      // X
                                     centerDist,                 // Y
                                     frameHeight + legLandHeight // Z
    );

    // Ending point of the curve
    rotateControlPoints[4] = Vector3(-joy1CurrentMagnitude, // X
                                     centerDist,            // Y
                                     frameHeight            // Z
    );

    Vector3 rotatePoint = GetPointOnBezierCurve(
        rotateControlPoints, fastMap(t, gait.cycleRatio, 1, 0, 1));
    //-------------------------------------------------------------------------------//

    // Return the weighted average of the two points
    return (straightPoint * abs(joy1CurrentMagnitude) +
            rotatePoint * abs(joy2CurrentVect.x)) /
           (abs(joy1CurrentMagnitude) + abs(joy2CurrentVect.x));
  }
}