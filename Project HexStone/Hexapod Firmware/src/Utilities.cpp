#include "Utilities.h"
#include <Arduino.h>
#include <math.h>

// Vector2 implementation
Vector2::Vector2() : x(0), y(0) {}
Vector2::Vector2(double x_, double y_) : x(x_), y(y_) {}

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

// Vector3 implementation
Vector3::Vector3() : x(0), y(0), z(0) {}
Vector3::Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

double Vector3::distanceTo(const Vector3 &other) const {
  double dx = x - other.x;
  double dy = y - other.y;
  double dz = z - other.z;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

Vector3 Vector3::operator-(const Vector3 &other) const {
  return Vector3(x - other.x, y - other.y, z - other.z);
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
  return String(x) + ", " + String(y) + ", " + String(z);
}

// Legtype implementation
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

double radToDeg(double radians) { return radians * double(180.0f / M_PI); }