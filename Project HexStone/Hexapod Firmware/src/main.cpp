#include "Controller.h"
#include "LegUtilities.h"
#include "Utilities.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

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
float frameHeight = 45;

// A multiplier for the global rotation of the hexapod
const float globalRotationFactor = 0.1;

// A multiplier for the global strafe of the hexapod
const float globalStrideFactor = 0.1;

// Distance from the center of the hexapod to the coxa of each leg
const int centerDist = 100;

// Multipliers for the stride rotation of each leg
const float strideMultiplier[6] = {-1, -1, -1, 1, 1, 1};

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

void initGait();
void standGait();
void loopGait();
void restGait();
// Given the current gait and a leg, this will return a vector representing the
// next position of the gait cycle
Vector3 getGaitCycle(const Gait &gait, Legtype leg);
void moveToPos(int leg, Vector3 pos);

/*************************************************************************/

Adafruit_PWMServoDriver pcaDriver = Adafruit_PWMServoDriver();

/*** TEMPERARY VARIABLE DECLARATION ***/
int coxaLen = 45.0f;     // Length of the coxa segment
int femurLen = 100.0f;   // Length of the femur segment
int tibiaLen = 180.0f;   // Length of the tibia segment
float bodyHeight = 50.0; // Height of the chassis, used for gait calculations
Legtype legs[NUM_LEGS];  // Placeholder for Servo Data
String servoNames[3] = {"Coxa", "Femur", "Tibia"}; // Names for each servo
// const Vector2 legOrigins[6] = { // For testing!
//     Vector2(-70, 85), Vector2(-70, 0), Vector2(-70, -85),
//     Vector2(70, -85), Vector2(70, 0),  Vector2(70, 85)}; // Leg origins
// const Vector2 gaitOrigins[6] = {
//     Vector2(-150, 200), Vector2(-170, 0), Vector2(-150, -200),
//     Vector2(150, -200), Vector2(170, 0),  Vector2(150, 200)}; // Gait origins
const Vector2 legOrigins[6] = {
    Vector2(0, 0), Vector2(0, 0), Vector2(0, 0),
    Vector2(0, 0), Vector2(0, 0), Vector2(0, 0),
};
const Vector2 gaitOrigins[6] = {Vector2(150, 0)};
bool debug = false; // Debug flag

Gait gait({0.0, 0.5, 0.0, 0.5, 0.0, 0.5}, // offsets
          0.5,                            // cycleRatio
          1.0,                            // speedFactor
          10.0,                           // liftHeight
          1.0,                            // strideLengthFactor
          50.0);

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

void setup() {
  Serial.begin(115200);
  while (!Serial) // Wait for serial port to initialize
    ;
  Serial.println("Single Leg Test");
  pcaDriver.begin();
  pcaDriver.setOscillatorFrequency(26000000);
  pcaDriver.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  pinMode(SERVOPIN_16, OUTPUT);     // Set the pin modes for the servos
  pinMode(SERVOPIN_17, OUTPUT);

  // --- Added for pin 4 toggle logic ---
  pinMode(19, OUTPUT); // Set pin 4 as output with pulldown
  // ------------------------------------

  for (int i = 0; i < NUM_LEGS; i++) { // Construct the leg objects
    legs[i].legNumber = i;
    legs[i].gaitOrigin = gaitOrigins[i];
    legs[i].legOrigin = legOrigins[i];
    legs[i].footPosition = gaitOrigins[i];
  }

  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  removePairedDevices(); // This helps to solve connection issues
  Serial.print("This device's MAC address is: ");
  printDeviceAddress();
  Serial.println("");

  delay(10);

  digitalWrite(19, HIGH);

  initGait(); // Initialize the gait mode for the first leg
}

// --- Added for pin 19 toggle logic ---
bool pin19State = true;
int debounceTime = 5;
// ------------------------------------

gaitMode gait_mode; // Instance of gaitMode

void loop() {

  // --- Added for pin 19 toggle logic ---
  ControllerData ctrl = getJoystickData();
  delay(1);
  if (ctrl.buttonCross && debounceTime > 5) {
    pin19State = !pin19State;
    digitalWrite(19, pin19State ? HIGH : LOW);
    debounceTime = 0;
  } else if (!ctrl.buttonCross) {
    debounceTime++;
  }

  // pcaDriver.setPWM(
  //     0, 0, float(fastMap(ctrl.leftStick.x, -127, 127, SERVOMIN, SERVOMAX)));
  // pcaDriver.setPWM(
  //     1, 0, float(fastMap(ctrl.leftStick.y, -127, 127, SERVOMIN, SERVOMAX)));
  // pcaDriver.setPWM(
  //     2, 0, float(fastMap(ctrl.rightStick.y, -127, 127, SERVOMIN,
  //     SERVOMAX)));
  setServoPositions(legs[0], inverseKinematics(legs[0], Vector3()));
  // ------------------------------------
}

void initGait() {
  for (int i = 0; i < 6; i++) {
    legStates[i] = Reset; // Reset all leg states
    cycleProgress[i] = gait.offsets[i] * cycleResolution;
  }
}

void standGait() {}

void loopGait() {
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

  forwardAmount = joy1CurrentMagnitude;
  turnAmount = joy2CurrentVect.x;

  moveToPos(0, getGaitPoint(0, cGait.pushFraction));
  moveToPos(1, getGaitPoint(1, cGait.pushFraction));
  moveToPos(2, getGaitPoint(2, cGait.pushFraction));
  moveToPos(3, getGaitPoint(3, cGait.pushFraction));
  moveToPos(4, getGaitPoint(4, cGait.pushFraction));
  moveToPos(5, getGaitPoint(5, cGait.pushFraction));

  float progressChangeAmount = max(abs(forwardAmount), abs(turnAmount)) *
                               cGait.gaitSpeedMult * globalSpeedMult *
                               potRightPercentage;

  // update the cycle progress for each leg
  for (int i = 0; i < 6; i++) {
    cycleProgress[i] += progressChangeAmount;

    // loop the cycle progress if it exceeds the points
    if (cycleProgress[i] >= points)
      cycleProgress[i] = cycleProgress[i] - points;
  }
}

void restGait() {}

Vector3 getGaitCycle(const Gait &gait, Legtype leg) {

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

  if (t < gait.cycleRatio) { // Pushing phase
    if (legStates[leg.legNumber] != Propelling)
      cycleStartPoints[leg.legNumber] = leg.footPosition;
    legStates[leg.legNumber] = Propelling;

    //----- Strafing Cycle -----//

    // Starting point of the strafing line
    vector<Vector3> strafeControlPoints = vector<Vector3>(2);
    strafeControlPoints[0] = cycleStartPoints[leg.legNumber];

    // Ending point of the strafing line
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

void moveToPos(Legtype leg, Vector3 pos) {
  Vector3 angles = inverseKinematics(leg.legOrigin, pos);
  setLegAngles(leg, angles);
}