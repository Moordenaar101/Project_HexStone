#pragma once

#include "Utilities.h"
#include <Adafruit_PWMServoDriver.h>

#define NUM_LEGS 6 // The number of legs to be initiated

// Servo PWM/Timing definitions
#define SERVOMIN 76  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 525 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN                                                                  \
  600 // Rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX                                                                  \
  2400 // Rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Because the PCA9685 only has 16 channels (Servo# 0-15), These pins are the
// remaining 2 servos that will be directly connected to the µC
#define SERVOPIN_16 16
#define SERVOPIN_17 17

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

Vector3 inverseKinematics(Vector2 legOrigin, const Vector3 &goal);

class gaitMode {
public:
  void init(const Gait &gait);
  void stand(const Gait &gait, Legtype leg);
  void loop(const Gait &gait, Legtype leg);
  void exit();
  Vector3 getGaitCycle(const Gait &gait, Legtype leg);

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
