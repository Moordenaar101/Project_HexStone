#include "Controller.h"
#include "LegUtilities.h"
#include "Utilities.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/****************************** Configuration  ******************************/

bool debug = false;   // Debug flag
bool doSerial = true; // Serial flag for debugging
bool needController =
    false;            // Wait for controller to connect before enabling servos
bool demoMode = true; // Demo mode flag to run a demo sequence

// The amount of position updates for a complete cycle
const int cycleResolution = 1000;

// The frequency of the main program
const unsigned long programFrequency = 50;                   // (Hz)
const unsigned long waitPeriod = 1000000 / programFrequency; // (µs)
unsigned long systemTime;
unsigned long previousSystemTime = 0;

/****************** Movement-Specific Configuration  ******************/

// How far the chassis is off the ground
int chassisHeight = 100;

// How hight to lift the legs when walking
int liftHeight = 100;

// The step length of the hexapod
int stepLength = 200;

// A multiplier for the global rotation of the hexapod
const float globalRotationFactor = 0.1;

// A multiplier for the global lift height of the hexapod
const float globalLiftFactor = 0.8;

// A multiplier for the global strafe of the hexapod
const float globalStrideFactor = 0.1;

// The landing height of each leg to prevent servo damage (Soft Land)
const float legLandHeight = 25;

/****************** Global Declarations  ******************/

// The current state of each leg in the gait cycle
int legStates[6];

// The current progress of each leg in the gait cycle
int cycleProgress[6];

// The current progress of each leg in the gait cycle
float tArray[6];

// Multipliers for the stride direction of each leg
const int strideMultiplier[6] = {1, 1, 1, -1, -1, -1};

// Multipliers for the rotation of each legs cycle
const int rotationMultiplier[6] = {1, 0, -1, 1, 0, -1};

enum modeSelection { assemble, packup, ikDemo, sit, stand, demoWalk, walk };

modeSelection currentMode = assemble;
modeSelection previousMode = assemble;

// Step counter for the current mode
float modeSteps = 0;

bool resetMode = false; // Flag to reset the mode

/****************** Hexapod-Specific Configuration  ******************/

// Leg Segment Lengths
float coxaLen = 45.0;
float femurLen = 100.0;
float tibiaLen = 180.0;

// The angle offset of each leg excluding the center legs
const float legAngle = 55;

// Offest used to ensure the leg will move away from the hexapod body as it
// lifts off the ground
const int legLiftClearance = 25;

// Assembly Position of All Legs (Angles)
const Vector3 assemblyPosition =
    radians(Vector3(90, 105, 180 - 160.01305)); // Arbitrary Values for now!
// const Vector3 assemblyPosition = Vector3(0, 101, 55);

// Packup Position of All Legs (Angles)
const Vector3 packupPosition =
    radians(Vector3(0, 105, 75)); // Arbitrary Values for now!

// sitting position of All Legs (Angles)
const Vector3 sitPosition =
    radians(Vector3(0, 105, 75)); // Arbitrary Values for now!

// movement speed factor for demo gaits
float demoWalkSpeed = 5.0;

// --- Servo toggle logic ---
bool pin19State = true;
unsigned long servoEnableDebounce = 1;

Adafruit_PWMServoDriver pcaDriver = Adafruit_PWMServoDriver();

/*** TEMPERARY VARIABLE DECLARATION ***/
Legtype legs[NUM_LEGS];                            // Placeholder for Servo Data
String servoNames[3] = {"Coxa", "Femur", "Tibia"}; // Names for each servo

// the base offset from (0,0) to where eachleg will rest
Vector2 gaitOrigin = Vector2(185, 0);

Gait gait({0.0, 0.5, 0.0, 0.5, 0.0, 0.5}, // offsets
          0.5,                            // cycleRatio
          0.25,                           // speedFactor
          80.0,                           // liftHeight
          1.0,                            // strideLengthFactor
          stepLength);                    // maxStrideLength

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

Servo Values: -> Probably Wrong!
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

void setup() {
  if (doSerial) {
    Serial.begin(115200);
    while (!Serial) // Wait for serial port to initialize
      ;
    Serial.println("Serial initialized");
  }

  // Initialize the PCA9685 driver
  pcaDriver.begin();
  pcaDriver.setOscillatorFrequency(26000000);
  pcaDriver.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  // --- ESP32 LED PWM setup for last two servos ---
  pinMode(LEG5FEMURPIN, OUTPUT); // Set the pin modes for the last two servos
  pinMode(LEG5TIBIAPIN, OUTPUT);
  ledcSetup(0, 50, 12); // channel 0, 50Hz, 12-bit resolution
  ledcAttachPin(LEG5FEMURPIN, 0);
  ledcSetup(1, 50, 12); // channel 1, 50Hz, 12-bit resolution
  ledcAttachPin(LEG5TIBIAPIN, 1);
  // ----------------------------------------------

  // PCA Driver output disable pin
  pinMode(19, OUTPUT);

  for (int i = 0; i < NUM_LEGS; i++) { // Construct the leg objects
    legs[i].legNumber = i;
    legs[i].footPosition = gaitOrigin;
  }

  if (doSerial) {
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisConnect);
  }

  // Initialize the PS4 controller
  PS4.begin();
  removePairedDevices(); // This helps to solve connection issues

  // If we need to wait for the controller to connect, lock the servos for now
  if (needController) {
    digitalWrite(19, HIGH);
  } else {
    digitalWrite(19, LOW);
  }

  // Temperary until the legs work
  // for (int i = 0; i < 6; i++) {
  //   legStates[i] = Reset; // Reset all leg states
  //   cycleProgress[i] = gait.offsets[i] * cycleResolution;
  // }
}

float t = 0;
bool reset = false;

void loop() {

  // Get the current time in microseconds
  systemTime = micros();

  // Skip if not enough time has passed since the last loop
  if (systemTime < previousSystemTime + waitPeriod) {
    return;
  }

  previousSystemTime = systemTime;

  // --- Added for pin 19 toggle logic ---
  ControllerData ctrl = getJoystickData();
  if (ctrl.buttonTouchpad && servoEnableDebounce > 5) {
    pin19State = !pin19State;
    digitalWrite(19, pin19State);
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

  // vector<Vector3> posBezierPoints = {Vector3(0, stepLength / 2, 0),
  //                                    Vector3(0, 0, liftHeight),
  //                                    Vector3(0, -stepLength / 2, 0)};

  // Serial.println(GetPointOnBezierCurve(posBezierPoints, t).toString());
  // setServoPositions(5, Vector3(90, 105, 180 - 160.01305)); // Assembly Vector

  // setServoPositions(
  //     0, Vector3(90, 105, 100).lerp(Vector3(90, 105, 180 - 160.01305), t));

  // t += 0.008;
  // t = t >= 1 ? 0 : t;

  // for (int i = 0; i < 6; i++) {
  //   setServoPositions(i, Vector3(0, 100, 90).lerp(Vector3(0, 90, 100), t));
  // }

  // setServoPositions(
  //     0, inverseKinematics(Vector3(0, 100, 0).lerp(Vector3(0, -100, 0), t)));
  // setServoPositions(
  //     1, inverseKinematics(Vector3(0, 100, 0).lerp(Vector3(0, -100, 0), t)));
  // setServoPositions(
  //     2, inverseKinematics(Vector3(0, 100, 0).lerp(Vector3(0, -100, 0), t)));

  // if (reset) {
  //   t += 0.001;
  //   reset = t >= 1 ? false : true;
  // } else {
  //   t -= 0.001;
  //   reset = t <= 0 ? true : false;
  // }

  // if (currentMode == assemble) {
  //   assemblyMode();
  // } else if (currentMode == packup) {
  //   packupMode();
  // } else if (currentMode == sit) {
  //   sitMode(modeSteps);
  // } else if (currentMode == stand) {
  //   standMode(modeSteps);
  // } else if (currentMode == ikDemo) {
  //   resetMode = ikDemoMode(modeSteps);
  //   modeSteps += demoWalkSpeed;
  // } else if (currentMode == demoWalk) {
  //   resetMode = walkingDemoMode(modeSteps);
  //   modeSteps += demoWalkSpeed;
  // } else if (currentMode == walk) {
  // }

  if (PS4.isConnected()) {
    if (inputDetected()) {
      walkGait(ctrl, gait);
    }
  }

  // walkGait(ctrl, gait);

  // pcaDriver.setPWM(15, 0, fastMap(90, 0, 180, SERVOMIN, SERVOMAX)); // Coxa

  // resetMode = walkingDemoMode(modeSteps);
  // modeSteps += demoWalkSpeed;
  // Serial.println(modeSteps);

  // Reset the steps counter
  // if (resetMode) {
  //   modeSteps = 0;
  // }
}