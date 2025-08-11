#include "LegUtilities.h"
#include "Utilities.h"
#include "controller.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

// Servo Offsets
const float coxaOffset = 90;
const float femurOffset = 0;
const float tibiaOffset = 0;

// Error vector for unreachable positions
Vector3 errorVector = Vector3(0, 60, 50);

Vector3 inverseKinematics(const Vector3 &goal) {

  // float goalDistance = Vector3(0, 0, 0).distanceTo(
  //     goal + Vector3(gaitOrigin.x, gaitOrigin.y, chassisHeight));
  // if (goalDistance > coxaLen + femurLen + tibiaLen) {
  //   Serial.println("Position out of reach!");
  //   // Serial.println(goal.distanceTo(Vector3()));
  //   // Serial.println((goal.lerp(Vector3(), (goal.distanceTo(Vector3()) /
  //   //                                       (coxaLen + femurLen + tibiaLen))
  //   -
  //   //                                          0.95))
  //   //                    .distanceTo(Vector3()));
  //   return goal.lerp(
  //       Vector3(),
  //       (goal.distanceTo(Vector3()) / (coxaLen + femurLen + tibiaLen)) -
  //       0.95);
  // }

  const float x = goal.x + gaitOrigin.x;
  const float y = goal.y + gaitOrigin.y;
  const float z = goal.z - chassisHeight;

  // Serial.println("Goal: " + goal.toString());

  float theta1 = radToDeg(atan2(y, x)); // Coxa

  float l = sqrt(sq(x) + sq(y)) - coxaLen;
  float h = sqrt(sq(l) + sq(z));
  float phi1 = acos(constrain(
      (sq(h) + sq(femurLen) - sq(tibiaLen)) / (2 * h * femurLen), -1, 1));

  float phi2 = atan2(z, l);

  float theta2 = radToDeg(phi1 + phi2); // Femur

  float theta3 = radToDeg(acos(constrain((sq(femurLen) + sq(tibiaLen) - sq(h)) /
                                             (2 * femurLen * tibiaLen),
                                         -1, 1))); // Tibia

  Vector3 angles;

  angles.x = theta1 + coxaOffset;        // Coxa
  angles.y = theta2 + femurOffset;       // Femur
  angles.z = 180 - theta3 + tibiaOffset; // Tibia

  return angles; // Return the angles in degrees
}

void setServoPositions(int legNum, Vector3 angles) {
  // This function sets the servo positions for the given leg object and angles
  // in degrees

  // Update the servo angles in the leg object
  legs[legNum].coxaAngle = angles.x;
  legs[legNum].femurAngle = angles.y;
  legs[legNum].tibiaAngle = angles.z;

  // Serial.println("Leg " + String(legNum) + " Angles:" + angles.toString());

  if (legNum <= 4) {
    pcaDriver.setPWM(legNum * 3 + 0, 0,
                     constrain(fastMap(angles.x, 0, 180, SERVOMIN, SERVOMAX),
                               SERVOMIN,
                               SERVOMAX)); // Coxa
    pcaDriver.setPWM(legNum * 3 + 1, 0,
                     constrain(fastMap(angles.y, 0, 180, SERVOMIN, SERVOMAX),
                               SERVOMIN,
                               SERVOMAX)); // Femur
    pcaDriver.setPWM(legNum * 3 + 2, 0,
                     constrain(fastMap(angles.z, 0, 180, SERVOMIN, SERVOMAX),
                               SERVOMIN, SERVOMAX)); // Tibia
  } else if (legNum == 5) {
    // For the last two servos, use ESP32 LED PWM
    pcaDriver.setPWM(legNum * 3 + 0, 0,
                     constrain(fastMap(angles.x, 0, 180, SERVOMIN, SERVOMAX),
                               SERVOMIN,
                               SERVOMAX)); // Coxa
    ledcWrite(0, constrain(fastMap(angles.y, 0, 180, SERVOMIN, SERVOMAX),
                           SERVOMIN, SERVOMAX)); // Femur
    ledcWrite(1,
              constrain(fastMap(angles.z, 0, 180, SERVOMIN, SERVOMAX), SERVOMIN,
                        SERVOMAX)); // Tibia
  }
}

void assemblyModeDelay() {
  for (int i = 0; i < NUM_LEGS; i++) {
    setServoPositions(i, assemblyPosition);
    delay(250);
  }
}

void assemblyMode() {
  for (int i = 0; i < NUM_LEGS; i++) {
    setServoPositions(i, assemblyPosition);
  }
}

void packupMode() {
  for (int i = 0; i < NUM_LEGS; i++) {
    setServoPositions(i, packupPosition);
    delay(250);
  }
}

void sitMode(int stepCount) {
  float zHeight;
  if (stepCount < 500) {
    zHeight = map(stepCount, 0, 500, chassisHeight, 0);
    for (int i = 0; i < NUM_LEGS; i++) {
      setServoPositions(i, inverseKinematics(Vector3(0, 0, zHeight)));
    }
  } else {
    for (int i = 0; i < NUM_LEGS; i++) {
      setServoPositions(i, inverseKinematics(Vector3(0, 0, chassisHeight)));
    }
  }
}

void standMode(int stepCount) {
  float zHeight;
  if (stepCount < 1000) {
    zHeight = map(stepCount, 0, 1000, 0, chassisHeight);
    for (int i = 0; i < NUM_LEGS; i++) {
      setServoPositions(i, inverseKinematics(Vector3(0, 0, zHeight)));
    }
  } else {
    for (int i = 0; i < NUM_LEGS; i++) {
      setServoPositions(i, inverseKinematics(Vector3(0, 0, 0)));
    }
  }
}

bool ikDemoMode(int stepCount) {
  bool done = false;
  for (int i = 0; i < NUM_LEGS; i++) {
    if (stepCount <= 500) { // -Y
      float t = fastMap(stepCount, 0, 500, 0, 1);
      setServoPositions(i, inverseKinematics(Vector3(0, 0, 0).lerp(
                               Vector3(0, -stepLength / 2, 0), t)));
      // Serial.println("-Y");

      // +Y
    } else if (stepCount > 500 && stepCount <= 1500) {
      float t = fastMap(stepCount, 501, 1500, 0, 1);
      setServoPositions(
          i, inverseKinematics(Vector3(0, -stepLength / 2, 0)
                                   .lerp(Vector3(0, stepLength / 2, 0), t)));
      // Serial.println("+Y");

      // Origin
    } else if (stepCount > 1500 && stepCount <= 2000) {
      float t = fastMap(stepCount, 1501, 2000, 0, 1);
      setServoPositions(
          i, inverseKinematics(
                 Vector3(0, stepLength / 2, 0).lerp(Vector3(0, 0, 0), t)));
      // Serial.println("Origin Y");

      // -X
    } else if (stepCount > 2000 && stepCount <= 2500) {
      float t = fastMap(stepCount, 2001, 2500, 0, 1);
      setServoPositions(i, inverseKinematics(Vector3(0, 0, 0).lerp(
                               Vector3(-stepLength / 2, 0, 0), t)));
      // Serial.println("-X");

      // +X
    } else if (stepCount > 2500 && stepCount <= 3500) {
      float t = fastMap(stepCount, 2501, 3500, 0, 1);
      setServoPositions(
          i, inverseKinematics(Vector3(-stepLength / 2, 0, 0)
                                   .lerp(Vector3(stepLength / 2, 0, 0), t)));
      // Serial.println("+X");

      // Origin
    } else if (stepCount > 3500 && stepCount <= 4000) {
      float t = fastMap(stepCount, 3500, 4000, 0, 1);
      setServoPositions(
          i, inverseKinematics(
                 Vector3(stepLength / 2, 0, 0).lerp(Vector3(0, 0, 0), t)));
      // Serial.println("Origin X");

      // -Z
    } else if (stepCount > 4000 && stepCount <= 4500) {
      float t = fastMap(stepCount, 4000, 4500, 0, 1);
      setServoPositions(i, inverseKinematics(Vector3(0, 0, 0).lerp(
                               Vector3(0, 0, -stepLength / 4), t)));
      // Serial.println("-Z");

      // +Z
    } else if (stepCount > 4500 && stepCount <= 5000) {
      float t = fastMap(stepCount, 4500, 5000, 0, 1);
      setServoPositions(
          i, inverseKinematics(Vector3(0, 0, -stepLength / 4)
                                   .lerp(Vector3(0, 0, stepLength / 4), t)));
      // Serial.println("+Z");

      // Origin
    } else if (stepCount > 5000) {
      float t = constrain(fastMap(stepCount, 5000, 6000, 0, 1), 0, 1);
      setServoPositions(
          i, inverseKinematics(
                 Vector3(0, 0, stepLength / 4).lerp(Vector3(0, 0, 0), t)));
      // Serial.println("Origin Z");
    }
  }
  done = stepCount >= 6000 ? true : false;
  return done;
}

bool walkingDemoMode(int stepCount) {
  bool done = false;

  vector<Vector3> posBezierPoints = {Vector3(0, stepLength / 2, 0),
                                     Vector3(0, 0, liftHeight),
                                     Vector3(0, -stepLength / 2, 0)};

  vector<Vector3> negBezierPoints = {Vector3(0, -stepLength / 2, 0),
                                     Vector3(0, 0, liftHeight),
                                     Vector3(0, stepLength / 2, 0)};

  if (stepCount <= 500) {
    float t = fastMap(stepCount, 0, 500, 0, 1);
    setServoPositions(
        0, inverseKinematics(
               (Vector3(0, stepLength / 2, 0)
                    .lerp(Vector3(0, -stepLength / 2, 0), t))
                   .rotate(legAngle * rotationMultiplier[0], Vector2())));

    setServoPositions(
        2, inverseKinematics(
               (Vector3(0, stepLength / 2, 0)
                    .lerp(Vector3(0, -stepLength / 2, 0), t))
                   .rotate(legAngle * rotationMultiplier[2], Vector2())));

    setServoPositions(
        4, inverseKinematics(
               (Vector3(0, -stepLength / 2, 0)
                    .lerp(Vector3(0, stepLength / 2, 0), t))
                   .rotate(legAngle * rotationMultiplier[4], Vector2())));

    setServoPositions(
        1, inverseKinematics(
               (GetPointOnBezierCurve(negBezierPoints, t))
                   .rotate(legAngle * rotationMultiplier[1], Vector2())));

    setServoPositions(
        3, inverseKinematics(
               (GetPointOnBezierCurve(posBezierPoints, t))
                   .rotate(legAngle * rotationMultiplier[3], Vector2())));

    setServoPositions(
        5, inverseKinematics(
               (GetPointOnBezierCurve(posBezierPoints, t))
                   .rotate(legAngle * rotationMultiplier[5], Vector2())));
  } else {
    float t = fastMap(stepCount, 500, 1000, 0, 1);
    setServoPositions(
        1, inverseKinematics(
               (Vector3(0, stepLength / 2, 0)
                    .lerp(Vector3(0, -stepLength / 2, 0), t))
                   .rotate(legAngle * rotationMultiplier[1], Vector2())));

    setServoPositions(
        3, inverseKinematics(
               (Vector3(0, -stepLength / 2, 0)
                    .lerp(Vector3(0, stepLength / 2, 0), t))
                   .rotate(legAngle * rotationMultiplier[3], Vector2())));

    setServoPositions(
        5, inverseKinematics(
               (Vector3(0, -stepLength / 2, 0)
                    .lerp(Vector3(0, stepLength / 2, 0), t))
                   .rotate(legAngle * rotationMultiplier[5], Vector2())));

    setServoPositions(
        0, inverseKinematics(
               (GetPointOnBezierCurve(negBezierPoints, t))
                   .rotate(legAngle * rotationMultiplier[0], Vector2())));

    setServoPositions(
        2, inverseKinematics(
               (GetPointOnBezierCurve(negBezierPoints, t))
                   .rotate(legAngle * rotationMultiplier[2], Vector2())));

    setServoPositions(
        4, inverseKinematics(
               (GetPointOnBezierCurve(posBezierPoints, t))
                   .rotate(legAngle * rotationMultiplier[4], Vector2())));
  }

  return stepCount >= 1000 ? true : false;
}

Vector2 joy1TargetVector;
float joy1TargetMagnitude;

Vector2 joy1CurrentVector;
float joy1CurrentMagnitude;

Vector2 joy2TargetVector;
float joy2TargetMagnitude;

Vector2 joy2CurrentVector;
float joy2CurrentMagnitude;

// The starting Position of each leg
Vector3 cycleStartPoints[6];

void walkGait(ControllerData controlData, Gait currentGait) {

  bool move = false;

  float tArray[6];

  joy1TargetVector.x = fastMap(controlData.leftStick.x, -127, 127, -100, 100);
  joy1TargetVector.y = fastMap(controlData.leftStick.y, -127, 127, -100, 100);
  joy2TargetVector.x = fastMap(controlData.rightStick.x, -127, 127, -100, 100);
  joy2TargetVector.y = fastMap(controlData.rightStick.y, -127, 127, -100, 100);

  // joy1TargetVector.x = fastMap(0, -127, 127, -100, 100);
  // joy1TargetVector.y = fastMap(127, -127, 127, -100, 100);
  // joy2TargetVector.x = fastMap(0, -127, 127, -100, 100);
  // joy2TargetVector.y = fastMap(0, -127, 127, -100, 100);

  joy1TargetMagnitude =
      constrain(hypot(joy1TargetVector.x, joy1TargetVector.y), 0, 100);

  joy2TargetMagnitude =
      constrain(hypot(joy2TargetVector.x, joy2TargetVector.y), 0, 100);

  joy1CurrentVector = joy1CurrentVector.lerp(joy1TargetVector, 0.04);
  joy1CurrentMagnitude = lerp(joy1CurrentMagnitude, joy1TargetMagnitude, 0.04);

  joy2CurrentVector = joy2CurrentVector.lerp(joy2TargetVector, 0.06);
  joy2CurrentMagnitude = lerp(joy2CurrentMagnitude, joy2TargetMagnitude, 0.06);

  for (int i = 0; i < 6; i++) {
    tArray[i] = (float)cycleProgress[i] / cycleResolution;
    // Serial.println("T: " + String(tArray[i]));
  };

  // Serial.println("Joystick Vectors: " + joy1CurrentVector.toString() + " , "
  // +
  //                joy2CurrentVector.toString());

  const float forwardAmount = joy1CurrentMagnitude;
  const float turnAmount = joy2CurrentVector.x;

  // Serial.println("Forward Amount: " + String(forwardAmount) +
  //                " Turn Amount: " + String(turnAmount)+"\n\n\n\n\n");

  // Serial.println("\n\n\n\n\n\n");

  // legs[3].footPosition =
  //     getGaitCycle(currentGait, legs[3], joy1CurrentVector,
  //                  joy1CurrentMagnitude, joy2CurrentVector, tArray[3]);
  // setServoPositions(legs[3].legNumber,
  // inverseKinematics(legs[3].footPosition));

  // legs[4].footPosition =
  //     getGaitCycle(currentGait, legs[4], joy1CurrentVector,
  //                  joy1CurrentMagnitude, joy2CurrentVector, tArray[4]);
  // setServoPositions(legs[4].legNumber,
  // inverseKinematics(legs[4].footPosition));

  for (int i = 0; i < NUM_LEGS; i++) { // Move each leg one gait cycle step
    legs[i].footPosition =
        getGaitCycle(currentGait, legs[i], joy1CurrentVector,
                     joy1CurrentMagnitude, joy2CurrentVector, tArray[i]);
    setServoPositions(legs[i].legNumber,
                      inverseKinematics(legs[i].footPosition));
  }

  // Serial.println(getGaitCycle(gait, legs[0]).toString());

  float progressChangeAmount =
      max(abs(forwardAmount), abs(turnAmount)) * currentGait.speedFactor;

  // update the cycle progress for each leg
  for (int i = 0; i < 6; i++) {
    cycleProgress[i] += progressChangeAmount;

    // loop the cycle progress if it exceeds the points
    if (cycleProgress[i] >= cycleResolution)
      cycleProgress[i] = cycleProgress[i] - cycleResolution;
  }
}

Vector3 getGaitCycle(const Gait &gait, Legtype leg, Vector2 j1Vect, float j1Mag,
                     Vector2 j2Vect, float t) {

  // The amount of rotation of the leg based on the right joysticks X axis
  const float rotationAmount = j2Vect.x * globalRotationFactor;

  // The strafing stride length of the leg based on the left joysticks Y axis
  Vector2 strafeStrideLength = j1Vect * gait.strideLengthFactor;
  strafeStrideLength.y =
      constrain(strafeStrideLength.y, -gait.maxStrideLength / 2,
                gait.maxStrideLength / 2);
  strafeStrideLength.x = constrain(strafeStrideLength.x, -gait.maxStrideLength,
                                   gait.maxStrideLength);

  if (t < gait.cycleRatio) { // Propelling phase
    if (legStates[leg.legNumber] != Propelling) {
      cycleStartPoints[leg.legNumber] = leg.footPosition;
      // Serial.println("\nPropelling: " + String(leg.legNumber));
    }
    legStates[leg.legNumber] = Propelling;

    //----- Strafing Cycle -----//

    // Starting point of the strafing line
    vector<Vector3> strafeControlPoints = vector<Vector3>(2);
    strafeControlPoints[0] = cycleStartPoints[leg.legNumber];
    strafeControlPoints[0].z = 0;

    // Serial.println("Strafe Stride Length: " +
    //                strafeStrideLength.toString());

    // Ending point of the strafing line
    strafeControlPoints[1] =
        Vector3(strafeStrideLength.x * strideMultiplier[leg.legNumber], // X
                strafeStrideLength.y * strideMultiplier[leg.legNumber], // Y
                0)                                                      // Z
            .rotate(legAngle * rotationMultiplier[leg.legNumber], Vector2());

    // Serial.println("Control Points: " + strafeControlPoints[0].toString() +
    //                " , " + strafeControlPoints[1].toString());

    Vector3 strafePoint = GetPointOnBezierCurve(
        strafeControlPoints, fastMap(t, 0, gait.cycleRatio, 0, 1));

    // Serial.println("Pushing Straife Point: " + strafePoint.toString());

    //-------------------------------------------------------------------------------//

    // Starting point of the curve
    vector<Vector3> rotateControlPoints = vector<Vector3>(3);
    rotateControlPoints[0] = cycleStartPoints[leg.legNumber];
    rotateControlPoints[0].z = 0;

    // Middle point of the curve
    rotateControlPoints[1] = Vector3();

    // Ending point of the curve
    rotateControlPoints[2] = Vector3(rotationAmount, // X
                                     0,              // Y
                                     0);             // Z

    Vector3 rotatePoint = GetPointOnBezierCurve(
        rotateControlPoints, fastMap(t, 0, gait.cycleRatio, 0, 1));

    // Serial.println("Points: " + strafePoint.toString() + " , " +
    //                rotatePoint.toString());
    //-------------------------------------------------------------------------------//

    // Serial.println(((strafePoint * abs(j1Mag) + rotatePoint * abs(j2Vect.x))
    // /
    //                 (abs(j1Mag) + abs(j2Vect.x)))
    //                    .toString());

    // Serial.println(strafePoint.toString());
    // return strafePoint;

    // Return the weighted average of the two points
    return (strafePoint * abs(j1Mag) + rotatePoint * abs(j2Vect.x)) /
           (abs(j1Mag) + abs(j2Vect.x));
  }

  // Lifting
  else {
    if (legStates[leg.legNumber] != Lifting) {
      cycleStartPoints[leg.legNumber] = leg.footPosition;
      // Serial.println("\nPropelling: " + String(leg.legNumber));
    }
    legStates[leg.legNumber] = Lifting;

    //------This cycle will cause the hexapod leg to lift up and return to the
    // beginning of the walk cycle in a straight line-----//

    // Starting point of the curve
    vector<Vector3> strafeControlPoints = vector<Vector3>(4);
    strafeControlPoints[0] = cycleStartPoints[leg.legNumber];

    // Control point directly above the starting point causing the leg to lift
    // up quickly
    strafeControlPoints[1] = cycleStartPoints[leg.legNumber] +
                             Vector3(0, 0, gait.liftHeight * globalLiftFactor);

    // Control point directly above the ending point preventing the leg from
    // running into the ground

    strafeControlPoints[2] =
        Vector3((-strafeStrideLength.x) * strideMultiplier[leg.legNumber], // X
                (-strafeStrideLength.y) * strideMultiplier[leg.legNumber], // Y
                legLandHeight                                              // Z
                )
            .rotate(legAngle * rotationMultiplier[leg.legNumber], Vector2());

    // Ending point of the curve
    strafeControlPoints[3] =
        Vector3((-strafeStrideLength.x) * strideMultiplier[leg.legNumber], // X
                (-strafeStrideLength.y) * strideMultiplier[leg.legNumber], // Y
                0)                                                         // Z
            .rotate(legAngle * rotationMultiplier[leg.legNumber], Vector2());

    Vector3 straightPoint = GetPointOnBezierCurve(
        strafeControlPoints, fastMap(t, gait.cycleRatio, 1, 0, 1));

    // Serial.println("Lifting Straight Point: " + straightPoint.toString());

    //-------------------------------------------------------------------------------//

    //------This cycle will cause the hexapod leg to lift up and return to the
    // beginning of the walk cycle in a curved line-----//

    // Starting point of the curve
    vector<Vector3> rotateControlPoints = vector<Vector3>(5);
    rotateControlPoints[0] = cycleStartPoints[leg.legNumber];

    // Control point directly above the starting point causing the leg to lift
    // up quickly
    rotateControlPoints[1] = cycleStartPoints[leg.legNumber] +
                             Vector3(0, 0, gait.liftHeight * globalLiftFactor);

    // Control point at the apex of the curve and offset away from the
    // hexapods body, cause the leg to lift up and away.
    rotateControlPoints[2] = Vector3(gaitOrigin.x,                      // X
                                     gaitOrigin.y + legLiftClearance,   // Y
                                     gait.liftHeight * globalLiftFactor // Z
    );

    // Control point directly above the ending point preventing the leg from
    // running into the ground
    rotateControlPoints[3] = Vector3(gaitOrigin.x,    // X
                                     -rotationAmount, // Y
                                     legLandHeight    // Z
    );

    // Ending point of the curve
    rotateControlPoints[4] = Vector3(gaitOrigin.x,    // X
                                     -rotationAmount, // Y
                                     0                // Z
    );

    Vector3 rotatePoint = GetPointOnBezierCurve(
        rotateControlPoints, fastMap(t, gait.cycleRatio, 1, 0, 1));

    // Serial.println("Lifting Rotate Point: " + rotatePoint.toString());
    //-------------------------------------------------------------------------------//

    // Serial.println("Final Lifting Point: " +
    //                ((straightPoint * abs(j1Mag) + rotatePoint *
    //                abs(j2Vect.x)) /
    //                 (abs(j1Mag) + abs(j2Vect.x)))
    //                    .toString() + "\n\n\n\n\n");

    // Return the weighted average of the two points
    return (straightPoint * abs(j1Mag) + rotatePoint * abs(j2Vect.x)) /
           (abs(j1Mag) + abs(j2Vect.x));
  }
}