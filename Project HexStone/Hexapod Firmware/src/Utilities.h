#pragma once
#include <Arduino.h>

struct Vector3; // Forward declaration

struct Vector2 {
  double x;
  double y;

  Vector2();
  Vector2(double x_, double y_);

  double distanceTo(const Vector2 &other) const;
  Vector2 lerp(const Vector2 &other, double t) const;
  Vector2 &operator=(const Vector3 &v3);
};

struct Vector3 {
  double x;
  double y;
  double z;

  Vector3();
  Vector3(double x_, double y_, double z_);

  double distanceTo(const Vector3 &other) const;
  Vector3 operator-(const Vector3 &other) const;
  double magnitude() const;
  Vector3 lerp(const Vector3 &other, double t) const;
  Vector3 &operator=(const Vector2 &v2);
  String toString() const;
};

struct Legtype { // Servo angles are from -90° → 90°
  float coxaAngle;
  float femurAngle;
  float tibiaAngle;
  int legNumber;
  Vector2 gaitOrigin;
  Vector2 legOrigin;
  Vector3 footPosition; // Current position of the foot in 3D space
  bool isGrounded;      // Whether the foot is currently on the ground

  Legtype();
  Legtype &operator=(const Legtype &other);
};

double radToDeg(double radians);
