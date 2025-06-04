#pragma once

#include "utilities.h"

// Global constants for robot geometry (should be defined in a .cpp file)
extern const float coxaLength;
extern const float femurLength;
extern const float tibiaLength;
extern const Vector3 legOrigin; // The base position of the leg in 3D space

// Optionally, add a norm() method to Vector3 for vector length
inline float vector3Norm(const Vector3 &v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Returns true if the goal is within the leg's reachable workspace
bool legReachable(int legIndex, const Vector3 &goal);

// Shifts the goal to within reachable workspace if needed
Vector3 shiftGoal(int legIndex, Vector3 goal);

// Calculates the servo angles for a given leg and goal position
void calcServoAngles(int leg, Vector3 goal);