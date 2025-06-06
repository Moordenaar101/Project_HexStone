#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

Adafruit_PWMServoDriver pcaDriver = Adafruit_PWMServoDriver();
#define SERVOMIN 75 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 525 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN                                                                  \
  600 // This is the rounded 'minimum' microsecond length based on the minimum
      // pulse of 150
#define USMAX                                                                  \
  2400 // This is the rounded 'maximum' microsecond length based on the maximum
       // pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define NUM_LEGS 1 // The number of legs to be initiated

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

  // Assignment from Vector3: assigns x, y, ignores z
  Vector2 &operator=(const Vector3 &v3) {
    x = v3.x;
    y = v3.y;
    return *this;
  }
};

struct Vector3 {
  float x;
  float y;
  float z;

  Vector3() : x(0), y(0), z(0) {}
  Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  // Assignment from Vector2: assigns x, y, sets z to 0
  Vector3 &operator=(const Vector2 &v2) {
    x = v2.x;
    y = v2.y;
    z = 0;
    return *this;
  }

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

struct Legtype {
  float coxaAngle;
  float femurAngle;
  float tibiaAngle;
  Vector2 gaitOrigin;
  Vector2 legOrigin;
  Vector3 footPosition; // Current position of the foot in 3D space
  bool isGrounded;      // Whether the foot is currently on the ground

  Legtype()
      : coxaAngle(90), femurAngle(90), tibiaAngle(90), gaitOrigin(),
        legOrigin(), footPosition(), isGrounded(true) {}

  // Assignment operator
  Legtype &operator=(const Legtype &other) {
    if (this != &other) {
      coxaAngle = other.coxaAngle;
      femurAngle = other.femurAngle;
      tibiaAngle = other.tibiaAngle;
      gaitOrigin = other.gaitOrigin;
      footPosition = other.footPosition;
      isGrounded = other.isGrounded;
    }
    return *this;
  }
};

/*** TEMPERARY VARIABLE DECLARATION ***/
int coxaLen = 45;       // Length of the coxa segment
int femurLen = 100;     // Length of the femur segment
int tibiaLen = 180;     // Length of the tibia segment
int bodyHeight = 50;    // Height of the chassis, used for gait calculations
Legtype legs[NUM_LEGS]; // Placeholder for Servo Data
String servoNames[3] = {"Coxa", "Femur", "Tibia"}; // Names for each servo
const Vector2 legOrigins[6] = {
    Vector2(-70, 85), Vector2(-70, 0), Vector2(-70, -85),
    Vector2(70, -85), Vector2(70, 0),  Vector2(70, 85)}; // Leg origins
const Vector2 gaitOrigins[6] = {
    Vector2(-150, 200), Vector2(-170, 0), Vector2(-150, -200),
    Vector2(150, -200), Vector2(170, 0),  Vector2(150, 200)}; // Gait origins
/**************************************/

// Hexapod Layout
/*
          Y
          ↑
          + → X

        Front
   (0)         (5)
     \  __↑__  /
      \/     \/
(1)___|       |___(4)
      |       |
      /\_____/\
     /         \
   (2)         (3)
        Back

 Parts:
 0 = Coxa
 1 = Femur
 2 = Tibia

          2
        // \\
 |0 == 1    \\
             \\
*/

void driveServos(Legtype leg, float goalAngles) {}

void setup1() {
  Serial.begin(9600);
  Serial.println("Single Leg Test");
  pcaDriver.begin();
  pcaDriver.setOscillatorFrequency(26000000);
  pcaDriver.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  for (int i = 0; i < NUM_LEGS; i++) { // Construct the leg objects
    legs[i].gaitOrigin = gaitOrigins[i];
    legs[i].legOrigin = legOrigins[i];
    legs[i].footPosition = gaitOrigins[i];
  }

  delay(10);
}

void loop1() {}

Vector3 inverseKinematics(Legtype leg, const Vector3 &goal) {

  const float xDistance = goal.x - leg.legOrigin.x;
  const float yDistance = goal.y - leg.legOrigin.y;
  const float rise = bodyHeight - goal.z;

  const float pivotToGoal =
      sqrt(xDistance * xDistance + yDistance * yDistance + rise * rise);

  const float theta = atan2(yDistance, xDistance);

  Vector3 effectiveGoal(goal.x - coxaLen * cos(theta),
                        goal.y - coxaLen * sin(theta), goal.z);

  // Calculate the position of the femur servo in 3d space
  // This will make checking if the goal is within reach easier
  const Vector3 femurServoPosition(
      leg.legOrigin.x + coxaLen * cos(leg.coxaAngle * M_PI / 180.0f),
      leg.legOrigin.y + coxaLen * sin(leg.coxaAngle * M_PI / 180.0f),
      bodyHeight);

  // Calculate the distance from the femur servo to the effective goal
  const float hipToGoalDistance = effectiveGoal.distanceTo(femurServoPosition);

  // Adjust goal if out of bounds
  if (hipToGoalDistance > femurLen + tibiaLen)
    effectiveGoal.lerp(femurServoPosition,
                       1 - (femurLen + tibiaLen - 0.001f) / hipToGoalDistance);

  //  accounting for the distance between the coxa and femur servos
  const Vector3 adjustedDistVect(effectiveGoal.x - femurServoPosition.x,
                                 effectiveGoal.y - femurServoPosition.y,
                                 femurServoPosition.z - effectiveGoal.z);

  const float h = sqrt(square(adjustedDistVect.x) + square(adjustedDistVect.y));
  const float hipToGoal = sqrt(square(h) + square(adjustedDistVect.z));

  return Vector3(
      atan2(adjustedDistVect.y, adjustedDistVect.x),
      acos((square(hipToGoal) + square(femurLen) - square(tibiaLen)) /
           (2 * hipToGoal * femurLen)) -
          atan(adjustedDistVect.z / h),
      acos((square(femurLen) + square(tibiaLen) - square(hipToGoal)) /
           (2 * femurLen * tibiaLen)));
}

// void setServoPositions(int leg, Vector3 angles) {
//   // This function sets the servo positions based on the calculated angles.

//   // Example of setting servo positions:
//   // pcaDriver.setPWM(leg * 3 + 0, 0, angleToPulseWidth(angles.x));
//   // pcaDriver.setPWM(leg * 3 + 1, 0, angleToPulseWidth(angles.y));
//   // pcaDriver.setPWM(leg * 3 + 2, 0, angleToPulseWidth(angles.z));
// }