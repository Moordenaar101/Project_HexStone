#pragma once
#include <Arduino.h>
#include <vector>

using std::vector;

struct Vector3; // Forward declaration

struct Vector2 {
  double x;
  double y;

  Vector2();
  Vector2(double x_, double y_);

  // Addition
  Vector2 operator+(const Vector2 &other) const;
  Vector2 operator+(double scalar) const;
  // Subtraction
  Vector2 operator-(const Vector2 &other) const;
  Vector2 operator-(double scalar) const;
  // Multiplication
  Vector2 operator*(const Vector2 &other) const;
  Vector2 operator*(double scalar) const;
  // Division
  Vector2 operator/(const Vector2 &other) const;
  Vector2 operator/(double scalar) const;

  double distanceTo(const Vector2 &other) const;
  Vector2 lerp(const Vector2 &other, double t) const;
  Vector2 &operator=(const Vector3 &v3);

  String toString() const; // Added toString() method
};

struct Vector3 {
  double x;
  double y;
  double z;

  Vector3();
  Vector3(double x_, double y_, double z_);

  // Addition
  Vector3 operator+(const Vector3 &other) const;
  Vector3 operator+(double scalar) const;
  // Subtraction
  Vector3 operator-(const Vector3 &other) const;
  Vector3 operator-(double scalar) const;
  // Multiplication
  Vector3 operator*(const Vector3 &other) const;
  Vector3 operator*(double scalar) const;
  // Division
  Vector3 operator/(const Vector3 &other) const;
  Vector3 operator/(double scalar) const;

  double distanceTo(const Vector3 &other) const;
  double magnitude() const;
  Vector3 lerp(const Vector3 &other, double t) const;
  Vector3 &operator=(const Vector2 &v2);
  String toString() const;
  Vector3 rotate(float angle, Vector2 pivot);
};

struct Legtype { // Servo angles are from -90° → 90°
  float coxaAngle;
  float femurAngle;
  float tibiaAngle;
  int legNumber;
  Vector3 footPosition; // Current position of the foot in 3D space

  Legtype();
  Legtype &operator=(const Legtype &other);
};

float lerp(float a, float b, float t);

double radToDeg(double radians);

float fastMap(float x, float in_min, float in_max, float out_min,
              float out_max);

Vector3 GetPointOnBezierCurve(vector<Vector3> &controlPoints, float t);
int binomialCoefficient(int n, int k);

inline std::vector<Vector3> operator+(const std::vector<Vector3> &arr,
                                      const Vector3 &v);

std::vector<Vector3> getOffsetBezierPoints(const int index);