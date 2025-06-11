#pragma once

#include "Utilities.h"

#define NUM_LEGS 1 // The number of legs to be initiated

// Servo PWM/Timing definitions
#define SERVOMIN 75  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 525 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN                                                                  \
  600 // Rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX                                                                  \
  2400 // Rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Because the PCA9685 only has 16 channels (Servo# 0-15), These pins are the
// remaining 2 servos that will be directly connected to the µC
#define SERVOPIN_16 5
#define SERVOPIN_17 6

Vector3 inverseKinematics(Legtype leg, const Vector3 &goal);
void setServoPositions(Legtype leg, Vector3 angles);
