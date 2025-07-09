#include "Utilities.h"
#include <Arduino.h>
#include <math.h>

// ===================== Vector2 Implementation =====================

Vector2::Vector2() : x(0), y(0) {}
Vector2::Vector2(double x_, double y_) : x(x_), y(y_) {}

// Addition
Vector2 Vector2::operator+(const Vector2 &other) const {
  return Vector2(x + other.x, y + other.y);
}
Vector2 Vector2::operator+(double scalar) const {
  return Vector2(x + scalar, y + scalar);
}

// Subtraction
Vector2 Vector2::operator-(const Vector2 &other) const {
  return Vector2(x - other.x, y - other.y);
}
Vector2 Vector2::operator-(double scalar) const {
  return Vector2(x - scalar, y - scalar);
}

// Multiplication
Vector2 Vector2::operator*(const Vector2 &other) const {
  return Vector2(x * other.x, y * other.y);
}
Vector2 Vector2::operator*(double scalar) const {
  return Vector2(x * scalar, y * scalar);
}

// Division
Vector2 Vector2::operator/(const Vector2 &other) const {
  return Vector2(x / other.x, y / other.y);
}
Vector2 Vector2::operator/(double scalar) const {
  return Vector2(x / scalar, y / scalar);
}

double Vector2::distanceTo(const Vector2 &other) const {
  double dx = x - other.x;
  double dy = y - other.y;
  return sqrt(dx * dx + dy * dy);
}

Vector2 Vector2::lerp(const Vector2 &other, double t) const {
  return Vector2(x + (other.x - x) * t, y + (other.y - y) * t);
}

Vector2 &Vector2::operator=(const Vector3 &v3) {
  x = v3.x;
  y = v3.y;
  return *this;
}

String Vector2::toString() const {
  return "(" + String(x) + ", " + String(y) + ")";
}

// ===================== Vector3 Implementation =====================

Vector3::Vector3() : x(0), y(0), z(0) {}
Vector3::Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

// Addition
Vector3 Vector3::operator+(const Vector3 &other) const {
  return Vector3(x + other.x, y + other.y, z + other.z);
}
Vector3 Vector3::operator+(double scalar) const {
  return Vector3(x + scalar, y + scalar, z + scalar);
}

// Subtraction
Vector3 Vector3::operator-(const Vector3 &other) const {
  return Vector3(x - other.x, y - other.y, z - other.z);
}
Vector3 Vector3::operator-(double scalar) const {
  return Vector3(x - scalar, y - scalar, z - scalar);
}

// Multiplication
Vector3 Vector3::operator*(const Vector3 &other) const {
  return Vector3(x * other.x, y * other.y, z * other.z);
}
Vector3 Vector3::operator*(double scalar) const {
  return Vector3(x * scalar, y * scalar, z * scalar);
}

// Division
Vector3 Vector3::operator/(const Vector3 &other) const {
  return Vector3(x / other.x, y / other.y, z / other.z);
}
Vector3 Vector3::operator/(double scalar) const {
  return Vector3(x / scalar, y / scalar, z / scalar);
}

double Vector3::distanceTo(const Vector3 &other) const {
  double dx = x - other.x;
  double dy = y - other.y;
  double dz = z - other.z;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

double Vector3::magnitude() const { return sqrt(x * x + y * y + z * z); }

Vector3 Vector3::lerp(const Vector3 &other, double t) const {
  return Vector3(x + (other.x - x) * t, y + (other.y - y) * t,
                 z + (other.z - z) * t);
}

Vector3 &Vector3::operator=(const Vector2 &v2) {
  x = v2.x;
  y = v2.y;
  z = 0;
  return *this;
}

String Vector3::toString() const {
  return "(" + String(x) + ", " + String(y) + ", " + String(z) + ")";
}

Vector3 Vector3::rotate(float angle, Vector2 pivot) {
  // Translate line so pivot point is at the origin
  if (angle == 0)
    return Vector3(x, y, z);

  x -= pivot.x;
  y -= pivot.y;
  float angleRad = radians(angle);

  // Rotate point by angle
  int x_rotated = x * cos(angleRad) - y * sin(angleRad);
  int y_rotated = x * sin(angleRad) + y * cos(angleRad);

  // Translate point back to original position
  x = x_rotated + pivot.x;
  y = y_rotated + pivot.y;

  return Vector3(x, y, z);
}

// ===================== Legtype Implementation =====================

Legtype::Legtype()
    : coxaAngle(0), femurAngle(0), tibiaAngle(0), legNumber(), gaitOrigin(),
      legOrigin(0, 0), footPosition(), isGrounded(true) {}

Legtype &Legtype::operator=(const Legtype &other) {
  if (this != &other) {
    coxaAngle = other.coxaAngle;
    femurAngle = other.femurAngle;
    tibiaAngle = other.tibiaAngle;
    legNumber = other.legNumber;
    gaitOrigin = other.gaitOrigin;
    legOrigin = other.legOrigin;
    footPosition = other.footPosition;
    isGrounded = other.isGrounded;
  }
  return *this;
}

// ===================== Utility Functions =====================

float lerp(float a, float b, float t) { return a * (1.0 - t) + (b * t); }

double radToDeg(double radians) { return radians * double(180.0f / M_PI); }

float fastMap(float x, float in_min, float in_max, float out_min,
              float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Vector3 GetPointOnBezierCurve(vector<Vector3> &controlPoints, float t) {
  Vector3 pos;
  int totalPoints = controlPoints.size(); // total number of control points
  if (totalPoints == 0)
    return pos; // Return zero vector if no points are provided

  for (int i = 0; i < totalPoints; i++) {
    float b = binomialCoefficient(totalPoints - 1, i) *
              pow(1 - t, totalPoints - 1 - i) * pow(t, i);
    pos.x += b * controlPoints[i].x;
    pos.y += b * controlPoints[i].y;
    pos.z += b * controlPoints[i].z;
  }

  return pos;
}

int binomialCoefficient(int n, int k) {
  int result = 1;

  // Calculate the binomial coefficient using the formula:
  // (n!) / (k! * (n - k)!)
  for (int i = 1; i <= k; i++) {
    result *= (n - (k - i));
    result /= i;
  }

  return result;
}