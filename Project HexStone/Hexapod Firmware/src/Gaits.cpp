#include "Gaits.h" // Include the header file

const float triOffsets[6] = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};
const float rippleOffsets[6] = {0.0f, 0.67f, 0.33f, 0.83f, 0.17f, 0.5f};
const float waveOffsets[6] = {0.0f, 0.17f, 0.33f, 0.83f, 0.67f, 0.5f};
const float quadOffsets[6] = {0.0f, 0.33f, 0.66f, 0.0f, 0.33f, 0.66f};
const float biOffsets[6] = {0.0f, 0.33f, 0.66f, 0.0f, 0.33f, 0.66f};
const float hopOffsets[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

Gait triGait(TRI, triOffsets, 0.55f, 1.1f, 1.1f, 100.0f, 200.0f);
Gait rippleGait(RIPPLE, rippleOffsets, 0.6f, 1.1f, 1.1f, 100.0f, 200.0f);
Gait waveGait(WAVE, waveOffsets, 0.82f, 0.50f, 1.4f, 130.0f, 200.0f);
Gait quadGait(QUAD, quadOffsets, 0.68f, 1.0f, 1.2f, 100.0f, 200.0f);
Gait biGait(BI, biOffsets, 0.35f, 2.0f, 1.1f, 160.0f, 200.0f);
Gait hopGait(HOP, hopOffsets, 0.55f, 1.1f, 1.4f, 80.0f, 225.0f);

Gait getGait(GaitType gaitType) {
  switch (gaitType) {
  case TRI:
    return triGait;
  case RIPPLE:
    return rippleGait;
  case WAVE:
    return waveGait;
  case QUAD:
    return quadGait;
  case BI:
    return biGait;
  case HOP:
    return hopGait;
  default:
    return triGait; // Default to TRI if no match found
  }
}
