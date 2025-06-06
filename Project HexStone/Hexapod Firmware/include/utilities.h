#pragma once

#include "Arduino.h"
#include <math.h>

struct Vector2 {
  float x;
  float y;

  Vector2() : x(0), y(0) {}
  Vector2(float x_, float y_) : x(x_), y(y_) {}

  float distanceTo(const Vector2 &other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return sqrt(dx * dx + dy * dy);
  }

  // Linear interpolation between this and another Vector2
  Vector2 lerp(const Vector2 &other, float t) const {
    return Vector2(x + (other.x - x) * t, y + (other.y - y) * t);
  }
};

struct Vector3 {
  float x;
  float y;
  float z;

  Vector3() : x(0), y(0), z(0) {}
  Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  float distanceTo(const Vector3 &other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    float dz = z - other.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
  }

  Vector3 operator-(const Vector3 &other) const {
    return Vector3(x - other.x, y - other.y, z - other.z);
  }

  float magnitude() const { return sqrt(x * x + y * y + z * z); }

  // Linear interpolation between this and another Vector3
  Vector3 lerp(const Vector3 &other, float t) const {
    return Vector3(x + (other.x - x) * t, y + (other.y - y) * t,
                   z + (other.z - z) * t);
  }
};

struct ServoData {
  int servoNum;
  String servoName;
  float currentAngle; // in degrees
  uint16_t pulseStart;
  uint16_t pulseEnd;

  // Default constructor
  ServoData()
      : servoNum(0), servoName(""), currentAngle(0.0f), pulseStart(0),
        pulseEnd(0) {}

  // Constructor with name
  ServoData(int num, String &name, float angle, uint16_t start, uint16_t end)
      : servoNum(num), servoName(name), currentAngle(angle), pulseStart(start),
        pulseEnd(end) {}

  // Calculate angle based on current pulse value
  float angleFromPulse(uint16_t pulse) const {
    // Linear mapping from pulse range to angle range
    return (pulse - pulseStart) / float(pulseEnd - pulseStart);
  }
};