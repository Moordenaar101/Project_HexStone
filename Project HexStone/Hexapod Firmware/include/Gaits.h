#ifndef GAITS_H
#define GAITS_H

#include <vector>
using std::vector;

// Gait types
enum GaitType {
  TRI,    // 0
  RIPPLE, // 1
  WAVE,   // 2
  QUAD,   // 3
  BI,     // 4
  HOP     // 5
};

// Single Gait struct definition
struct Gait {
  GaitType gaitType;
  float cycleOffsetPercentages[6];
  float pushFraction;
  float gaitSpeedMult;
  float strideLengthMult;
  float liftHeight;
  float maxStrideLength;

  Gait(GaitType gt, const float co[6], float pf, float gsm, float slm, float lh,
       float msl)
      : gaitType(gt), pushFraction(pf), gaitSpeedMult(gsm),
        strideLengthMult(slm), liftHeight(lh), maxStrideLength(msl) {
    for (int i = 0; i < 6; ++i)
      cycleOffsetPercentages[i] = co[i];
  }
};

// The actual objects are defined in Gaits.cpp
extern Gait triGait;
extern Gait rippleGait;
extern Gait waveGait;
extern Gait quadGait;
extern Gait biGait;
extern Gait hopGait;

Gait getGait(GaitType gaitType);

#endif // GAITS_H