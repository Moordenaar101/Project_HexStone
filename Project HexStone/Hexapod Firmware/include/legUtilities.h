#pragma once
#include "Utilities.h"
#include "controller.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

#define NUM_LEGS 6 // The number of legs to be initiated

// Servo PWM/Timing definitions
#define SERVOMIN 77  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 525 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN                                                                  \
  600 // Rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX                                                                  \
  2400 // Rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Because the PCA9685 only has 16 channels (Servo# 0-15), These pins are the
// remaining 2 servos that will be directly connected to the µC
#define LEG5FEMURPIN 32
#define LEG5TIBIAPIN 33

// Extern variables from main.cpp
extern float coxaLen;
extern float femurLen;
extern float tibiaLen;
extern bool debug;
extern Adafruit_PWMServoDriver pcaDriver;
extern Legtype legs[NUM_LEGS];
extern Vector2 gaitOrigin;
extern const Vector3 assemblyPosition;
extern const Vector3 packupPosition;
extern const Vector3 sitPosition;
extern const Vector3 restPosition;
extern int stepLength;
extern int chassisHeight;
extern int liftHeight;
extern const int cycleResolution;
extern int cycleProgress[6];
extern const float globalRotationFactor;
extern int legStates[6];
extern const int strideMultiplier[6];
extern const int rotationMultiplier[6];
extern const float legAngle;
extern const float globalLiftFactor;
extern const float legLandHeight;
extern const int legLiftClearance;

// Single Gait struct definition
struct Gait {
  std::vector<float> offsets;
  float cycleRatio;
  float speedFactor;
  float liftHeight;
  float strideLengthFactor;
  float maxStrideLength;

  Gait(const std::vector<float> &offsets_, float cycleRatio, float speedFactor,
       float liftHeight, float strideLength, float maxStrideLength)
      : offsets(offsets_), cycleRatio(cycleRatio), speedFactor(speedFactor),
        liftHeight(liftHeight), strideLengthFactor(strideLength),
        maxStrideLength(maxStrideLength) {}
};

enum LegState { Propelling, Lifting, Standing, Reset };

Vector3 inverseKinematics(const Vector3 &goal);

void setServoPositions(int legNum, Vector3 angles);
void assemblyModeDelay();
void assemblyMode();
void packupMode();
void sitMode(int stepCount);
void standMode(int stepCount);
bool ikDemoMode(int stepCount);
bool walkingDemoMode(int stepCount);
void walkGait(ControllerData controlData, Gait currentGait);
Vector3 getGaitCycle(const Gait &gait, Legtype leg, Vector2 j1Vect, float j1Mag,
                     Vector2 j2Vect, float t);

class gaitMode {
public:
  void init(const Gait &gait);
  void stand(const Gait &gait, Legtype leg);
  void loop(const Gait &gait, Legtype leg);
  void exit();
  Vector3 getGaitCycle(const Gait &gait, Legtype leg, Vector2 j1Vect,
                       float j1Mag, Vector2 j2Vect, float t);

private:
  int points = 1000;
  Vector2 joy1TargetVect;
  float joy1TargetMagnitude;

  Vector2 joy1CurrentVect;
  float joy1CurrentMagnitude;

  Vector2 joy2TargetVect;
  float joy2TargetMagnitude;

  Vector2 joy2CurrentVect;
  float joy2CurrentMagnitude;
};