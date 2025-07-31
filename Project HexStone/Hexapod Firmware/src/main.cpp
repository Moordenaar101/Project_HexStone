#include "Controller.h"
#include "LegUtilities.h"
#include "Utilities.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/****************************** Configuration  ******************************/

bool debug = false;    // Debug flag
bool doSerial = false; // Serial flag for debugging
bool waitForController =
    false;            // Wait for controller to connect before enabling servos
bool demoMode = true; // Demo mode flag to run a demo sequence

/****************** Movement-Specific Global Declarations  ******************/

// The amount of position updates for a complete cycle
const int cycleResolution = 1000;

// The current state of each leg in the gait cycle
int legStates[6];

// The current progress of each leg in the gait cycle
int cycleProgress[6];

// The start points of the gait cycle for each leg
Vector3 cycleStartPoints[6];

// The current progress of each leg in the gait cycle
float tArray[6];

// The Current height of the hexapod from the ground
float frameHeight = 75;

// A multiplier for the global rotation of the hexapod
const float globalRotationFactor = 0.1;

// A multiplier for the global lift height of the hexapod
const float globalLiftFactor = 0.8;

// A multiplier for the global strafe of the hexapod
const float globalStrideFactor = 0.1;

// A currently arbetrary value. Using it until I don't need it or figure out
// what it does
const int centerDist = 150;

// Multipliers for the stride rotation of each leg
const float strideMultiplier[6] = {1, 1, 1, -1, -1, -1};

// Multipliers for the rotation of each legs cycle
const float rotationMultiplier[6] = {1, 0, -1, 1, 0, -1};

// The landing height of each leg to prevent servo damage (Soft Land)
const float legLandHeight = 25;

// The angle offset of each leg excluding the center legs
const float legPlacementAngle = 55;

// The magnitude of the Left joysticks Y axis
float forwardAmount;

// The value of the Right joysticks X axis
float turnAmount;

// The number of points in the gait cycle (Resolution)
int points = 1000;

// Left Joystick Actual Vector
Vector2 joy1TargetVect;

// Left Joysticks Actual Magnitude
float joy1TargetMagnitude;

// Left Joysticks Current Vector
Vector2 joy1CurrentVect;

// Left Joysticks Current Magnitude
float joy1CurrentMagnitude;

// Right Joysticks Actual Vector
Vector2 joy2TargetVect;

// Right Joysticks Actual Magnitude
float joy2TargetMagnitude;

// Right Joysticks Current Vector
Vector2 joy2CurrentVect;

// Right Joysticks Current Magnitude
float joy2CurrentMagnitude;

// Offest used to ensure the leg will move away from the hexapod body as it
// lifts off the ground
Vector2 legLiftClearanceVect = Vector2(25, 25);

// Assembly Position of All Legs
const Vector3 assemblyPosition = radians(Vector3(0, 101, 55));

// -63.09341 X
// -90.10672 Y

// Assembly position in coordinates
const Vector3 assemblyPositionCoords[6] = {Vector3(-133.09341, 90.10672, -66),
                                           Vector3(-210, 0, -66),
                                           Vector3(-133.09341, -90.10672, -66),
                                           Vector3(133.09341, -90.10672, -66),
                                           Vector3(110, 0, -66),
                                           Vector3(133.09341, 90.10672, -66)};

// Rest Position of All Legs
const Vector3 restPosition = radians(Vector3(0, 118, 2));

// Variables specific to Demo Mode
unsigned long selectionDebounce = 5;
unsigned long demoTimer = 0;
bool demoIK = false;
bool demoWalking = false;
unsigned long currentMillis = 0;
enum DemoModeSelection { packup, ikDemo, sit, stand, walk };
int steps = 0;

DemoModeSelection currentDemo = packup;

float walkSpeed = 0.0075;

// --- Servo toggle logic ---
bool pin19State = true;
unsigned long servoEnableDebounce = 5;

void initGait();
void standGait();
void loopGait();
void restGait();
void demoStand(int stepCount);
void calibrationMode();
// Given the current gait and a leg, this will return a vector representing the
// next position of the gait cycle
Vector3 getGaitCycle(const Gait &gait, Legtype leg);
// void moveToPos(Legtype leg, Vector3 pos);

/*************************************************************************/

Adafruit_PWMServoDriver pcaDriver = Adafruit_PWMServoDriver();

gaitMode gait_mode; // Instance of gaitMode

/*** TEMPERARY VARIABLE DECLARATION ***/
float coxaLen = 45.0;    // Length of the coxa segment
float femurLen = 100.0;  // Length of the femur segment
float tibiaLen = 180.0;  // Length of the tibia segment
float bodyHeight = 75.0; // Height of the chassis, used for gait calculations
Legtype legs[NUM_LEGS];  // Placeholder for Servo Data
String servoNames[3] = {"Coxa", "Femur", "Tibia"}; // Names for each servo
const Vector2 legOrigins[6] = {                    // For testing!
    Vector2(-70, 85), Vector2(-100, 0), Vector2(-70, -85),
    Vector2(70, -85), Vector2(100, 0),  Vector2(70, 85)}; // Leg origins
const Vector2 gaitOrigins[6] = {Vector2(-127.35764, 166.9152),
                                Vector2(-200, 0),
                                Vector2(-127.35764, -166.9152),
                                Vector2(127.35764, -166.9152),
                                Vector2(200, 0),
                                Vector2(127.35764, 166.9152)}; // Gait origins

const Vector3 standingPositions[6] = {
    Vector3(-127.35764, 166.9152, -75),
    Vector3(-200, 0, -75),
    Vector3(-127.35764, -166.9152, -75),
    Vector3(127.35764, -166.9152, -75),
    Vector3(200, 0, -75),
    Vector3(127.35764, 166.9152, -75)}; // Standing Positions

Gait gait({0.0, 0.5, 0.0, 0.5, 0.0, 0.5}, // offsets
          0.5,                            // cycleRatio
          0.25,                           // speedFactor
          50.0,                           // liftHeight
          1.0,                            // strideLengthFactor
          100.0);                         // maxStrideLength

float test = 0.0;
bool flip = true;

vector<Vector3> vectPointsA = {Vector3(0, 75, 0), Vector3(0, 75, 25),
                               Vector3(0, 0, 75), Vector3(0, -75, 25),
                               Vector3(0, -75, -75)};

vector<Vector3> vectPointsB = {Vector3(200, 100, -50), Vector3(200, 100, 0),
                               Vector3(200, 0, 100), Vector3(200, -100, 0),
                               Vector3(200, -100, -50)};

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

Assembly Coordinates: (110, 0, -60)

Servo Values:
- Femur IRL
  Max: 120°
  Min: 0°
  Cal: 101°

- Femur IK
  Max: -60°
  Min: 65°
  Cal: 105°

- Tibia IRL
  Max: 0°
  Min: 180°
  Cal: 55°

- Tibia IK
  Max: 195°
  Min: 30°
  Cal: 255°
*/

void setServoPositions(int legNum, Vector3 angles, bool debug = false) {
  // This function sets the servo positions for the given leg object

  // Update the servo angles in the leg object
  legs[legNum].coxaAngle = angles.x;
  legs[legNum].femurAngle = angles.y;
  legs[legNum].tibiaAngle = angles.z;

  angles = degrees(angles);

  if (!debug) {
    if (legNum <= 4) {

      pcaDriver.setPWM(
          legNum * 3 + 0, 0,
          constrain(fastMap(angles.x + 90, 0, 180, SERVOMIN, SERVOMAX),
                    SERVOMIN, SERVOMAX)); // Coxa
      pcaDriver.setPWM(
          legNum * 3 + 1, 0,
          constrain(fastMap(angles.y - 4, 0, 180, SERVOMIN, SERVOMAX), SERVOMIN,
                    SERVOMAX)); // Femur
      pcaDriver.setPWM(
          legNum * 3 + 2, 0,
          constrain(fastMap(angles.z + 20, 0, 180, SERVOMIN, SERVOMAX),
                    SERVOMIN, SERVOMAX)); // Tibia
    } else {
      // For the last two servos, use ESP32 LED PWM
      pcaDriver.setPWM(
          legNum * 3 + 0, 0,
          constrain(fastMap(angles.x + 90, 0, 180, SERVOMIN, SERVOMAX),
                    SERVOMIN, SERVOMAX)); // Coxa
      ledcWrite(
          0, constrain(fastMap(angles.y - 4, 0, 180, 0, 255), 0, 255)); // Femur
      ledcWrite(1, constrain(fastMap(angles.z + 20, 0, 180, 0, 255), 0,
                             255)); // Tibia
    }
  } else {
    angles.x = radToDeg(angles.x);
    angles.y = radToDeg(angles.y);
    angles.z = radToDeg(angles.z);

    // Print the angles to the serial monitor
    Serial.print(">> Leg: ");
    Serial.print(legNum);
    Serial.print(" [Coxa: ");
    Serial.print(angles.x);
    Serial.print("] [Femur: ");
    Serial.print(angles.y);
    Serial.print("] [Tibia: ");
    Serial.print(angles.z);
    Serial.println("]");
  }
}

void setup() {
  if (doSerial) {
    Serial.begin(115200);
    while (!Serial) // Wait for serial port to initialize
      ;
    Serial.println("Single Leg Test");
  }

  pcaDriver.begin();
  pcaDriver.setOscillatorFrequency(26000000);
  pcaDriver.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  pinMode(LEG5FEMURPIN, OUTPUT);    // Set the pin modes for the last two servos
  pinMode(LEG5TIBIAPIN, OUTPUT);

  // --- ESP32 LED PWM setup for last two servos ---
  ledcSetup(0, 50, 8); // channel 0, 50Hz, 8-bit resolution
  ledcAttachPin(LEG5FEMURPIN, 0);
  ledcSetup(1, 50, 8); // channel 1, 50Hz, 8-bit resolution
  ledcAttachPin(LEG5TIBIAPIN, 1);
  // ----------------------------------------------

  // --- PCA Driver output disable pin ---
  pinMode(19, OUTPUT); // Set pin 19 as output with pulldown
  // ------------------------------------

  for (int i = 0; i < NUM_LEGS; i++) { // Construct the leg objects
    legs[i].legNumber = i;
    legs[i].gaitOrigin = gaitOrigins[i];
    legs[i].legOrigin = legOrigins[i];
    legs[i].footPosition = gaitOrigins[i];
  }

  if (doSerial) {
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisConnect);
  }

  PS4.begin();
  removePairedDevices(); // This helps to solve connection issues

  delay(10);

  if (waitForController) {
    digitalWrite(19, HIGH);
  } else {
    digitalWrite(19, LOW);
  }

  initGait(); // Initialize the gait mode for the first leg

  calibrationMode(); // Packup mode to reset all servos

  currentMillis = millis(); // Initialize the currentMillis variable
}

void loop() {
  // --- Added for pin 19 toggle logic ---
  ControllerData ctrl = getJoystickData();
  if (ctrl.buttonTouchpad && servoEnableDebounce > 5) {
    pin19State = !pin19State;
    digitalWrite(19, pin19State ? HIGH : LOW);
    servoEnableDebounce = 0;
  } else if (ctrl.buttonR1 && servoEnableDebounce > 5) {
    setServoPositions(0, assemblyPosition);
    setServoPositions(1, assemblyPosition);
    setServoPositions(2, assemblyPosition);
    setServoPositions(3, assemblyPosition);
    setServoPositions(4, assemblyPosition);
    setServoPositions(5, assemblyPosition);
    while (!ctrl.buttonR1)
      delay(50);
    while (!getJoystickData().buttonL1) { // Pauses all functions
      delay(100);
    }
    servoEnableDebounce = 0;
  } else if (!ctrl.buttonTouchpad) {
    servoEnableDebounce++;
  }
  if (ctrl.buttonOptions) {
    if (doSerial)
      Serial.println("Press Triangle to exit");
    while (!ctrl.buttonOptions)
      delay(50);
    while (!getJoystickData().buttonOptions) { // Pauses all functions
      delay(100);
    }
  }

  // if (PS4.isConnected() && !demoMode) {
  //   loopGait();
  // } else if (PS4.isConnected() && demoMode) {
  //   if (currentMillis > currentMillis + selectionDebounce) {
  //     currentMillis = millis();
  //     if (ctrl.buttonTriangle) {
  //       currentDemo = stand;
  //     } else if (ctrl.buttonCircle) {
  //       currentDemo = ikDemo;
  //     } else if (ctrl.buttonSquare) {
  //       currentDemo = sit;
  //     } else if (ctrl.buttonCross) {
  //       currentDemo = walk;
  //     } else if (ctrl.buttonShare) {
  //       currentDemo = packup;
  //     }
  //   }

  // if (currentDemo == packup) {
  //   restMode();
  // }

  // if (ctrl.buttonCross && millis() > demoTimer + 100) {
  //   demoTimer = millis();
  //   demoWalking = !demoWalking;
  // }

  if (demoIK) {
    steps += 0.1;
    if (steps >= 6500) {
      steps = 0;
    }
    ikDemoMode(steps);
  }

  if (demoWalking) {
    steps += 0.1;
    if (steps >= 6500) {
      steps = 0;
    }
    demoWalkingMode(steps);
  }

  // if (flip) {
  //   test += walkSpeed;
  //   flip = test >= 1 ? false : true;
  //   setServoPositions(
  //       0, inverseKinematics(
  //              legs[0].legOrigin,
  //              Vector3(-200, 100, -50).lerp(Vector3(-200, -100, -50),
  //              test)));

  //   setServoPositions(
  //       2, inverseKinematics(
  //              legs[2].legOrigin,
  //              Vector3(-200, 100, -50).lerp(Vector3(-200, -100, -50),
  //              test)));

  //   setServoPositions(
  //       4, inverseKinematics(
  //              legs[4].legOrigin,
  //              Vector3(200, 100, -50).lerp(Vector3(200, -100, -50),
  //              test)));

  //   setServoPositions(
  //       1, inverseKinematics(legs[1].legOrigin,
  //                            GetPointOnBezierCurve(vectPointsA, test)));

  //   setServoPositions(
  //       3, inverseKinematics(legs[3].legOrigin,
  //                            GetPointOnBezierCurve(vectPointsB, test)));

  //   setServoPositions(
  //       5, inverseKinematics(legs[5].legOrigin,
  //                            GetPointOnBezierCurve(vectPointsB, test)));
  // } else {
  //   test -= walkSpeed;
  //   flip = test <= 0 ? true : false;
  //   setServoPositions(
  //       1, inverseKinematics(
  //              legs[1].legOrigin,
  //              Vector3(-200, 100, -50).lerp(Vector3(-200, -100, -50),
  //              test)));

  //   setServoPositions(
  //       3, inverseKinematics(
  //              legs[3].legOrigin,
  //              Vector3(200, 100, -50).lerp(Vector3(200, -100, -50),
  //              test)));

  //   setServoPositions(
  //       5, inverseKinematics(
  //              legs[5].legOrigin,
  //              Vector3(200, 100, -50).lerp(Vector3(200, -100, -50),
  //              test)));

  //   setServoPositions(
  //       0, inverseKinematics(legs[0].legOrigin,
  //                            GetPointOnBezierCurve(vectPointsA, test)));

  //   setServoPositions(
  //       2, inverseKinematics(legs[2].legOrigin,
  //                            GetPointOnBezierCurve(vectPointsA, test)));

  //   setServoPositions(
  //       4, inverseKinematics(legs[4].legOrigin,
  //                            GetPointOnBezierCurve(vectPointsB, test)));
  // }
  // }
}

void initGait() {
  for (int i = 0; i < 6; i++) {
    legStates[i] = Reset; // Reset all leg states
    cycleProgress[i] = gait.offsets[i] * cycleResolution;
  }
}

void standGait() {}

void loopGait() {
  ControllerData joyData = getJoystickData();
  if (joyData.leftStick.x > 8 || joyData.leftStick.x < -8) {
    joy1TargetVect.x = (float)map(joyData.leftStick.x, -127, 127, -100, 100);
  } else {
    joy1TargetVect.x = 0;
  }
  if (joyData.leftStick.y > 8 || joyData.leftStick.y < -8) {
    joy1TargetVect.y = (float)map(joyData.leftStick.y, -127, 127, -100, 100);
  } else {
    joy1TargetVect.y = 0;
  }

  joy1TargetMagnitude =
      constrain(hypot(joy1TargetVect.x, joy1TargetVect.y), 0, 100);

  joy2TargetVect = {(float)map(joyData.rightStick.x, -127, 127, -100, 100),
                    (float)map(joyData.rightStick.y, -127, 127, -100, 100)};

  joy2TargetMagnitude =
      constrain(hypot(joy2TargetVect.x, joy2TargetVect.y), 0, 100);

  joy1CurrentVect = joy1CurrentVect.lerp(joy1TargetVect, 0.04);

  joy1CurrentMagnitude = lerp(joy1CurrentMagnitude, joy1TargetMagnitude, 0.04);

  joy2CurrentVect = joy2CurrentVect.lerp(joy2TargetVect, 0.06);
  joy2CurrentMagnitude = lerp(joy2CurrentMagnitude, joy2TargetMagnitude, 0.06);

  for (int i = 0; i < 6; i++) {
    tArray[i] = (float)cycleProgress[i] / cycleResolution;
  };

  forwardAmount = joy1CurrentMagnitude;
  turnAmount = joy2CurrentVect.x;

  // for (int i = 0; i < NUM_LEGS; i++) { // Move each leg one gait cycle step
  //   legs[i].footPosition = getGaitCycle(gait, legs[i]);
  //   moveToPos(legs[i], legs[i].footPosition);
  // }

  // Serial.println(getGaitCycle(gait, legs[0]).toString());

  legs[0].footPosition = getGaitCycle(gait, legs[0]);
  // Serial.println(
  //     inverseKinematics(legs[0].legOrigin,
  //     legs[0].footPosition).toString());
  // Serial.println(legs[0].footPosition.toString());

  float progressChangeAmount =
      max(abs(forwardAmount), abs(turnAmount)) * gait.speedFactor;

  // update the cycle progress for each leg
  for (int i = 0; i < 6; i++) {
    cycleProgress[i] += progressChangeAmount;

    // loop the cycle progress if it exceeds the points
    if (cycleProgress[i] >= points)
      cycleProgress[i] = cycleProgress[i] - points;
  }

  legs[4].footPosition = getGaitCycle(gait, legs[4]);
  setServoPositions(4,
                    inverseKinematics(legs[4].legOrigin, legs[4].footPosition));
}

void restGait() {}

Vector3 getGaitCycle(const Gait &gait,
                     Legtype leg) { // Aiming for ~(-180, 240, 0)

  // The amount of rotation of the leg based on the right joysticks X axis
  float rotationAmount = joy2CurrentVect.x * globalRotationFactor;

  // The strafing stride length of the leg based on the left joysticks Y axis
  Vector2 strafeStrideLength = joy1CurrentVect * gait.strideLengthFactor;
  strafeStrideLength.y =
      constrain(strafeStrideLength.y, -gait.maxStrideLength / 2,
                gait.maxStrideLength / 2);
  strafeStrideLength.x = constrain(strafeStrideLength.x, -gait.maxStrideLength,
                                   gait.maxStrideLength);

  // The current progress of the gait cycle for the leg
  float t = tArray[leg.legNumber];

  if (t < gait.cycleRatio) { // Propelling phase
    if (legStates[leg.legNumber] != Propelling)
      cycleStartPoints[leg.legNumber] = leg.footPosition;
    legStates[leg.legNumber] = Propelling;

    //----- Strafing Cycle -----//

    // Starting point of the strafing line
    vector<Vector3> strafeControlPoints = vector<Vector3>(2);
    strafeControlPoints[0] = cycleStartPoints[leg.legNumber];
    strafeControlPoints[0].z = 0;

    // Ending point of the strafing line
    // strafeControlPoints[1] =
    // Vector3(leg.gaitOrigin.x + strafeStrideLength.y *
    // strideMultiplier[leg.legNumber], // X
    //         -strafeStrideLength.x * strideMultiplier[leg.legNumber] +
    //             centerDist, // Y
    //         0                                              // Z
    //         )
    //     .rotate(legPlacementAngle * rotationMultiplier[leg.legNumber],
    //             Vector2(0, centerDist));

    strafeControlPoints[1] = Vector3((leg.gaitOrigin.x + strafeStrideLength.x) *
                                         strideMultiplier[leg.legNumber], // X
                                     (leg.gaitOrigin.y + strafeStrideLength.y) *
                                         strideMultiplier[leg.legNumber], // Y
                                     0                                    // Z
    );

    Vector3 strafePoint = GetPointOnBezierCurve(
        strafeControlPoints, fastMap(t, 0, gait.cycleRatio, 0, 1));

    // Serial.println("\n\nStrafeControlPoints:\nT = " +
    //                String(fastMap(t, 0, gait.cycleRatio, 0, 1)) + "\n" +
    //                strafeControlPoints[0].toString() + "\n" +
    //                strafeControlPoints[1].toString() +
    //                "\n(0, 0, 0)\n(0, 0, 0)" +
    //                "\nPoint: " + strafePoint.toString() +
    //                "\nSS Length: " + strafeStrideLength.toString());
    // delay(100);

    //-------------------------------------------------------------------------------//

    // Starting point of the curve
    vector<Vector3> rotateControlPoints = vector<Vector3>(3);
    rotateControlPoints[0] = cycleStartPoints[leg.legNumber];

    // Middle point of the curve
    rotateControlPoints[1] = Vector3(leg.gaitOrigin.x, // X
                                     0,                // Y
                                     0                 // Z
    );

    // Ending point of the curve
    rotateControlPoints[2] = Vector3(leg.gaitOrigin.x + rotationAmount, // X
                                     0,                                 // Y
                                     0                                  // Z
    );

    Vector3 rotatePoint = GetPointOnBezierCurve(
        rotateControlPoints, fastMap(t, 0, gait.cycleRatio, 0, 1));
    //-------------------------------------------------------------------------------//

    strafePoint.z = 0; // Ensure the strafing point is on the ground
    rotatePoint.z = 0; // Ensure the rotation point is on the ground

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
    strafeControlPoints[1] = cycleStartPoints[leg.legNumber] +
                             Vector3(cycleStartPoints[leg.legNumber].x,
                                     cycleStartPoints[leg.legNumber].y,
                                     gait.liftHeight * globalLiftFactor);

    // Control point directly above the ending point preventing the leg from
    // running into the ground
    // strafeControlPoints[2] =
    //     Vector3(strafeStrideLength.y * strideMultiplier[leg.legNumber] +
    //                 gaitOrigins[leg.legNumber].x, // X
    //             strafeStrideLength.x * strideMultiplier[leg.legNumber] +
    //                 centerDist + gaitOrigins[leg.legNumber].y, // Y
    //             0 + legLandHeight                              // Z
    //             )
    //         .rotate(legPlacementAngle * rotationMultiplier[leg.legNumber],
    //                 Vector2(0, centerDist));

    strafeControlPoints[2] =
        Vector3((gaitOrigins[leg.legNumber].x + strafeStrideLength.x) *
                    strideMultiplier[leg.legNumber], // X
                (gaitOrigins[leg.legNumber].y + strafeStrideLength.y) *
                    strideMultiplier[leg.legNumber], // Y
                0 + legLandHeight                    // Z
        );

    // Ending point of the curve
    // strafeControlPoints[3] =
    //     Vector3(-strafeStrideLength.y * strideMultiplier[leg.legNumber] +
    //                 gaitOrigins[leg.legNumber].x,
    //             strafeStrideLength.x * strideMultiplier[leg.legNumber] +
    //                 centerDist + gaitOrigins[leg.legNumber].y,
    //             0)
    //         .rotate(legPlacementAngle * rotationMultiplier[leg.legNumber],
    //                 Vector2(0, centerDist));

    strafeControlPoints[3] =
        Vector3((gaitOrigins[leg.legNumber].x + strafeStrideLength.x) *
                    strideMultiplier[leg.legNumber], // X
                (gaitOrigins[leg.legNumber].y + strafeStrideLength.y) *
                    strideMultiplier[leg.legNumber], // Y
                0);                                  // Z

    Vector3 straightPoint = GetPointOnBezierCurve(
        strafeControlPoints, fastMap(t, gait.cycleRatio, 1, 0, 1));

    // Serial.println("\n\nStrafeControlPoints:\nT = " +
    //                String(fastMap(t, gait.cycleRatio, 1, 0, 1)) + "\n" +
    //                strafeControlPoints[0].toString() + "\n" +
    //                strafeControlPoints[1].toString() + "\n" +
    //                strafeControlPoints[2].toString() + "\n" +
    //                strafeControlPoints[3].toString() +
    //                "\nPoint: " + straightPoint.toString() +
    //                "\nSS Length: " + strafeStrideLength.toString());
    // delay(100);

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
    rotateControlPoints[2] = Vector3(
        leg.gaitOrigin.x + (legLiftClearanceVect.x *
                            (leg.gaitOrigin.x / abs(leg.gaitOrigin.x))), // X
        leg.gaitOrigin.y + (legLiftClearanceVect.y *
                            (leg.gaitOrigin.y / abs(leg.gaitOrigin.y))), // Y
        0 + gait.liftHeight * globalLiftFactor                           // Z
    );

    // Control point directly above the ending point preventing the leg from
    // running into the ground
    rotateControlPoints[3] =
        Vector3(leg.gaitOrigin.x + joy1CurrentMagnitude, // X
                leg.gaitOrigin.y,                        // Y
                legLandHeight                            // Z
        );

    // Ending point of the curve
    rotateControlPoints[4] =
        Vector3(leg.gaitOrigin.x + joy1CurrentMagnitude, // X
                leg.gaitOrigin.y,                        // Y
                0                                        // Z
        );

    Vector3 rotatePoint = GetPointOnBezierCurve(
        rotateControlPoints, fastMap(t, gait.cycleRatio, 1, 0, 1));
    //-------------------------------------------------------------------------------//

    // Serial.println("Straight Point: " + straightPoint.toString() +
    //                "\nRotate Point: " + rotatePoint.toString());

    // Return the weighted average of the two points
    return (straightPoint * abs(joy1CurrentMagnitude) +
            rotatePoint * abs(joy2CurrentVect.x)) /
           (abs(joy1CurrentMagnitude) + abs(joy2CurrentVect.x));
  }
}

// void moveToPos(Legtype leg, Vector3 pos) {
//   Vector3 angles = inverseKinematics(leg.legOrigin, pos);
//   setServoPositions(leg.legNumber, angles, true);
// }

// const Vector2 gaitOrigins[6] = {Vector2(-127.35764, 166.9152),
//                                 Vector2(-200, 0),
//                                 Vector2(-127.35764, -166.9152),
//                                 Vector2(127.35764, -166.9152),
//                                 Vector2(200, 0),
//                                 Vector2(127.35764, 166.9152)}; // Gait
//                                 origins
// }

int stepLength = 75;

void demoWalkingMode(float increment) {
  flip = increment >= 1000 ? false : true;

  if (flip) {
    float t = fastMap(increment, 0, 1000, 0, 1);
    setServoPositions(
        0, inverseKinematics(
               legs[0].legOrigin,
               Vector3(-127.35764, 166.9152 + stepLength, -50)
                   .lerp(Vector3(-127.35764, 166.9152 - stepLength, -50), t)));

    setServoPositions(
        2, inverseKinematics(
               legs[2].legOrigin,
               Vector3(-127.35764, -166.9152 + stepLength, -50)
                   .lerp(Vector3(-127.35764, -166.9152 - stepLength, -50), t)));

    setServoPositions(
        4, inverseKinematics(
               legs[4].legOrigin,
               Vector3(200, 75, -50).lerp(Vector3(200, -75, -50), t)));

    setServoPositions(1,
                      inverseKinematics(legs[1].legOrigin,
                                        GetPointOnBezierCurve(vectPointsA, t)));

    setServoPositions(3,
                      inverseKinematics(legs[3].legOrigin,
                                        GetPointOnBezierCurve(vectPointsB, t)));

    setServoPositions(5,
                      inverseKinematics(legs[5].legOrigin,
                                        GetPointOnBezierCurve(vectPointsB, t)));
  } else {
    float t = fastMap(increment, 1000, 2000, 0, 1);
    setServoPositions(
        1, inverseKinematics(
               legs[1].legOrigin,
               Vector3(-200, 75, -50).lerp(Vector3(-200, -75, -50), t)));

    setServoPositions(
        3,
        inverseKinematics(legs[3].legOrigin,
                          Vector3(127.35764, -166.9152 + stepLength, -50)
                              .lerp(Vector3(170, -160 - stepLength, -50), t)));

    setServoPositions(
        5, inverseKinematics(
               legs[5].legOrigin,
               Vector3(127.35764, 166.9152 + stepLength, -50)
                   .lerp(Vector3(127.35764, 166.9152 - stepLength, -50), t)));

    getAdjustedBezierPoints(0);

    setServoPositions(0,
                      inverseKinematics(legs[0].legOrigin,
                                        GetPointOnBezierCurve(vectPointsA, t)));

    setServoPositions(2,
                      inverseKinematics(legs[2].legOrigin,
                                        GetPointOnBezierCurve(vectPointsA, t)));

    setServoPositions(4,
                      inverseKinematics(legs[4].legOrigin,
                                        GetPointOnBezierCurve(vectPointsB, t)));
  }
}

vector<Vector3> getAdjustedBezierPoints(int index) {
  for (int i = 0; i < vectPointsA.size(); i++) {
    vectPointsA[i] = vectPointsA[i] + standingPositions[index];
  }
  return vectPointsA;
}

void calibrationMode() {
  for (int i = 0; i < NUM_LEGS; i++) {
    setServoPositions(i,
                      assemblyPosition); // Set all servos to assembly position
    delay(1000);
  }
}

void restMode() {
  for (int i = 0; i < NUM_LEGS; i++) {
    setServoPositions(i, restPosition);
    delay(1000);
  }
}

void ikDemoMode(float stepCount) {

  for (int i = 0; i < NUM_LEGS; i++) {
    if (stepCount <= 500) {
      float t = fastMap(stepCount, 0, 500, 0, 1);
      setServoPositions(
          i, inverseKinematics(legs[i].legOrigin,
                               Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -50)
                                   .lerp(Vector3(gaitOrigins[i].x,
                                                 gaitOrigins[i].y - 100, -50),
                                         t)));
    } else if (stepCount > 500 && stepCount <= 1500) {
      float t = fastMap(stepCount, 501, 1500, 0, 1);
      setServoPositions(
          i,
          inverseKinematics(
              legs[i].legOrigin,
              Vector3(gaitOrigins[i].x, gaitOrigins[i].y - 100, -50)
                  .lerp(Vector3(gaitOrigins[i].x, gaitOrigins[i].y + 100, -50),
                        t)));

    } else if (stepCount > 1500 && stepCount <= 2000) {
      float t = fastMap(stepCount, 1501, 2000, 0, 1);
      setServoPositions(
          i,
          inverseKinematics(
              legs[i].legOrigin,
              Vector3(gaitOrigins[i].x, gaitOrigins[i].y + 100, -50)
                  .lerp(Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -50), t)));

    } else if (stepCount > 2000 && stepCount <= 2500) {
      float t = fastMap(stepCount, 2001, 2500, 0, 1);
      setServoPositions(
          i, inverseKinematics(legs[i].legOrigin,
                               Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -50)
                                   .lerp(Vector3(gaitOrigins[i].x - 75,
                                                 gaitOrigins[i].y, -50),
                                         t)));
    } else if (stepCount > 2500 && stepCount <= 3500) {
      float t = fastMap(stepCount, 2501, 3500, 0, 1);
      setServoPositions(
          i,
          inverseKinematics(
              legs[i].legOrigin,
              Vector3(gaitOrigins[i].x - 75, gaitOrigins[i].y, -50)
                  .lerp(Vector3(gaitOrigins[i].x + 75, gaitOrigins[i].y, -50),
                        t)));
    } else if (stepCount > 3500 && stepCount <= 4000) {
      float t = fastMap(stepCount, 3500, 4000, 0, 1);
      setServoPositions(
          i,
          inverseKinematics(
              legs[i].legOrigin,
              Vector3(gaitOrigins[i].x + 75, gaitOrigins[i].y, -50)
                  .lerp(Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -50), t)));
    } else if (stepCount > 4000 && stepCount <= 4500) {
      float t = fastMap(stepCount, 4000, 4500, 0, 1);
      setServoPositions(
          i,
          inverseKinematics(
              legs[i].legOrigin,
              Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -50)
                  .lerp(Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -100), t)));
    } else if (stepCount > 4500 && stepCount <= 5000) {
      float t = fastMap(stepCount, 4500, 5000, 0, 1);
      setServoPositions(
          i, inverseKinematics(
                 legs[i].legOrigin,
                 Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -100)
                     .lerp(Vector3(gaitOrigins[i].x, gaitOrigins[i].y, 0), t)));
    } else if (stepCount > 5000 && stepCount <= 6000) {
      float t = fastMap(stepCount, 5000, 6000, 0, 1);
      setServoPositions(
          i, inverseKinematics(
                 legs[i].legOrigin,
                 Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -100)
                     .lerp(Vector3(gaitOrigins[i].x, gaitOrigins[i].y, 0), t)));
    } else if (stepCount > 5000 && stepCount <= 6000) {
      float t = fastMap(stepCount, 5000, 6000, 0, 1);
      setServoPositions(
          i, inverseKinematics(
                 legs[i].legOrigin,
                 Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -100)
                     .lerp(Vector3(gaitOrigins[i].x, gaitOrigins[i].y, 0), t)));
    } else if (stepCount > 6000 && stepCount <= 6500) {
      float t = fastMap(stepCount, 6000, 6500, 0, 1);
      setServoPositions(
          i,
          inverseKinematics(
              legs[i].legOrigin,
              Vector3(gaitOrigins[i].x, gaitOrigins[i].y, 0)
                  .lerp(Vector3(gaitOrigins[i].x, gaitOrigins[i].y, -50), t)));
    }
  }
}

void demoStand(int stepCount) {
  float zHeight = 0;

  if (stepCount < 500) {
    zHeight = map(stepCount, 0, 500, 0, -100);
    for (int i = 0; i < NUM_LEGS; i++) {
      setServoPositions(i,
                        inverseKinematics(legs[i].legOrigin,
                                          Vector3(gaitOrigins[i].x,
                                                  gaitOrigins[i].y, zHeight)));
    }
  } else {
    for (int i = 0; i < NUM_LEGS; i++) {
      setServoPositions(i, inverseKinematics(legs[i].legOrigin,
                                             Vector3(gaitOrigins[i].x,
                                                     gaitOrigins[i].y, -100)));
    }
  }
}