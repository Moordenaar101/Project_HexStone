#include "Controller.h"
#include "LegUtilities.h"
#include "Utilities.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ps5Controller.h>

/****************************** Configuration  ******************************/

bool debug = false;   // Debug flag
bool doSerial = true; // Serial flag for debugging
bool needController =
    false;            // Wait for controller to connect before enabling servos
bool demoMode = true; // Demo mode flag to run a demo sequence

// The amount of position updates for a complete cycle
const int cycleResolution = 1000;

// The frequency of the main program
const unsigned long programFrequency = 100;                  // (Hz)
const unsigned long waitPeriod = 1000000 / programFrequency; // (µs)
unsigned long systemTime;
unsigned long previousSystemTime = 0;

/****************** Movement-Specific Configuration  ******************/

// How far the chassis is off the ground
int chassisHeight = 100;

// How hight to lift the legs when walking
int liftHeight = 100;

// The step length of the hexapod
int stepLength = 125;

// A multiplier for the global rotation of the hexapod
const float globalRotationFactor = 0.8;

// A multiplier for the global lift height of the hexapod
const float globalLiftFactor = 1.0;

// A multiplier for the global strafe of the hexapod
const float globalStrideFactor = 0.2;

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

bool reset = false; // Flag to reset the mode

// Controller button debounce timing array
static unsigned long lastDebounceTimes[17] = {0}; // 17 binary buttons

// Debounce delay for button presses
const unsigned long debounceDelay = 50; // ms

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
const Vector3 assemblyPosition = (Vector3(90, 105, 19.98695));

// Packup Position of All Legs (Angles)
const Vector3 packupPosition =
    radians(Vector3(0, 105, 160)); // Arbitrary Values for now!

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

  // if (doSerial) {
  //   ps5.attachOnConnect(onConnect);
  //   ps5.attachOnDisconnect(onDisConnect);
  // }

  // Initialize the ps5 controller
  // ps5.begin();
  // removePairedDevices(); // This helps to solve connection issues

  ps5.begin("F8:9E:94:5D:F7:AE");

  // If we need to wait for the controller to connect, lock the servos for now
  if (needController) {
    digitalWrite(19, HIGH);
  } else {
    digitalWrite(19, LOW);
  }

  // Temperary until the legs work
  for (int i = 0; i < 6; i++) {
    legStates[i] = Reset; // Reset all leg states
    cycleProgress[i] = gait.offsets[i] * cycleResolution;
  }

  assemblyModeDelay();
}

void loop() {

  // Get the current time in microseconds
  systemTime = micros();

  // Skip if not enough time has passed since the last loop
  if (systemTime < previousSystemTime + waitPeriod) {
    return;
  }

  previousSystemTime = systemTime;

  // setServoPositions(1, assemblyPosition);

  // --- Added for pin 19 toggle logic ---
  // ControllerData ctrl = getJoystickData();
  // if (ctrl.buttonTouchpad && servoEnableDebounce > 5) {
  //   pin19State = !pin19State;
  //   digitalWrite(19, pin19State);
  //   servoEnableDebounce = 0;
  // } else if (ctrl.buttonR1 && servoEnableDebounce > 5) {
  //   setServoPositions(0, assemblyPosition);
  //   setServoPositions(1, assemblyPosition);
  //   setServoPositions(2, assemblyPosition);
  //   setServoPositions(3, assemblyPosition);
  //   setServoPositions(4, assemblyPosition);
  //   setServoPositions(5, assemblyPosition);
  //   while (!ctrl.buttonR1)
  //     delay(50);
  //   while (!getJoystickData().buttonL1) { // Pauses all functions
  //     delay(100);
  //   }
  //   servoEnableDebounce = 0;
  // } else if (!ctrl.buttonTouchpad) {
  //   servoEnableDebounce++;
  // }
  // if (ctrl.buttonOptions) {
  //   if (doSerial)
  //     Serial.println("Press Triangle to exit");
  //   while (!ctrl.buttonOptions)
  //     delay(50);
  //   while (!getJoystickData().buttonOptions) { // Pauses all functions
  //     delay(100);
  //   }
  // }
  Serial.println(ps5.isConnected());

  if (ps5.isConnected()) {

    ControllerData ctrl = getJoystickData();
    unsigned long now = millis();

    // setServoPositions(5, assemblyPosition);

    // if (!inputDetected()) {
    //   walkGait(ctrl, gait);
    // }

    // Touchpad button
    if (ctrl.buttonTouchpad) {
      if (now - lastDebounceTimes[0] > debounceDelay) {
        lastDebounceTimes[0] = now + debounceDelay; // Double the debounce delay
        pin19State = !pin19State;
        digitalWrite(19, pin19State);
      }
    }

    // Options button
    if (ctrl.buttonOptions) {
      if (now - lastDebounceTimes[1] > debounceDelay) {
        lastDebounceTimes[1] = now;
        currentMode = assemble;
      }
    }

    // Share button
    if (ctrl.buttonShare) {
      if (now - lastDebounceTimes[2] > debounceDelay) {
        lastDebounceTimes[2] = now;
        // --- Code for share button press ---
      }
    }

    // Cross button
    if (ctrl.buttonCross) {
      if (now - lastDebounceTimes[3] > debounceDelay) {
        lastDebounceTimes[3] = now;
        currentMode = walk;
      }
    }

    // Circle button
    if (ctrl.buttonCircle) {
      if (now - lastDebounceTimes[4] > debounceDelay) {
        lastDebounceTimes[4] = now;
        currentMode = ikDemo;
      }
    }

    // Square button
    if (ctrl.buttonSquare) {
      if (now - lastDebounceTimes[5] > debounceDelay) {
        lastDebounceTimes[5] = now;
        currentMode = demoWalk;
      }
    }

    // Triangle button
    if (ctrl.buttonTriangle) {
      if (now - lastDebounceTimes[6] > debounceDelay) {
        lastDebounceTimes[6] = now;
      }
    }

    // L1 button
    if (ctrl.buttonL1) {
      if (now - lastDebounceTimes[7] > debounceDelay) {
        lastDebounceTimes[7] = now;
        // --- Code for L1 button press ---
      }
    }

    // R1 button
    if (ctrl.buttonR1) {
      if (now - lastDebounceTimes[8] > debounceDelay) {
        lastDebounceTimes[8] = now;
        // --- Code for R1 button press ---
      }
    }

    // L2 button
    if (ctrl.buttonL2) {
      if (now - lastDebounceTimes[9] > debounceDelay) {
        lastDebounceTimes[9] = now;
        // --- Code for L2 button press ---
      }
    }

    // R2 button
    if (ctrl.buttonR2) {
      if (now - lastDebounceTimes[10] > debounceDelay) {
        lastDebounceTimes[10] = now;
        // --- Code for R2 button press ---
      }
    }

    // Up button
    if (ctrl.buttonUp) {
      if (now - lastDebounceTimes[11] > debounceDelay) {
        lastDebounceTimes[11] = now;
        currentMode = stand;
      }
    }

    // Down button
    if (ctrl.buttonDown) {
      if (now - lastDebounceTimes[12] > debounceDelay) {
        lastDebounceTimes[12] = now;
        currentMode = sit;
      }
    }

    // Left button
    if (ctrl.buttonLeft) {
      if (now - lastDebounceTimes[13] > debounceDelay) {
        lastDebounceTimes[13] = now;
        currentMode = packup;
      }
    }

    // Right button
    if (ctrl.buttonRight) {
      if (now - lastDebounceTimes[14] > debounceDelay) {
        lastDebounceTimes[14] = now;
        // --- Code for right button press ---
      }
    }
  }

  // --- Mode selection logic ---
  if (currentMode == assemble) {
    assemblyMode();
  } else if (currentMode == packup) {
    packupMode();
  } else if (currentMode == sit) {
    sitMode(modeSteps);
    modeSteps += demoWalkSpeed;
  } else if (currentMode == stand) {
    standMode(modeSteps);
    modeSteps += demoWalkSpeed;
  } else if (currentMode == ikDemo) {
    reset = ikDemoMode(modeSteps);
    modeSteps += demoWalkSpeed;
  } else if (currentMode == demoWalk) {
    reset = walkingDemoMode(modeSteps);
    modeSteps += demoWalkSpeed;
  } else if (currentMode == walk) {
    if (inputDetected()) {
      walkGait(getJoystickData(), gait);
    }
  }

  if (reset) {
    modeSteps = 0;
  }
}