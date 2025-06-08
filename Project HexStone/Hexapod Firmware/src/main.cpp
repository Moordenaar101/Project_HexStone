#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Adafruit_PWMServoDriver pcaDriver = Adafruit_PWMServoDriver();
#define SERVOMIN 75  // This is the 'minimum' pulse length count (out of 4096)
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
  double x;
  double y;

  Vector2() : x(0), y(0) {}
  Vector2(double x_, double y_) : x(x_), y(y_) {}

  double distanceTo(const Vector2 &other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    return sqrt(dx * dx + dy * dy);
  }

  // Linear interpolation between this and another Vector2
  Vector2 lerp(const Vector2 &other, double t) const {
    return Vector2(x + (other.x - x) * t, y + (other.y - y) * t);
  }

  // Declare assignment from Vector3
  Vector2 &operator=(const struct Vector3 &v3);
};

struct Vector3 {
  double x;
  double y;
  double z;

  Vector3() : x(0), y(0), z(0) {}
  Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  double distanceTo(const Vector3 &other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    double dz = z - other.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
  }

  Vector3 operator-(const Vector3 &other) const {
    return Vector3(x - other.x, y - other.y, z - other.z);
  }

  double magnitude() const { return sqrt(x * x + y * y + z * z); }

  // Linear interpolation between this and another Vector3
  Vector3 lerp(const Vector3 &other, double t) const {
    return Vector3(x + (other.x - x) * t, y + (other.y - y) * t,
                   z + (other.z - z) * t);
  }

  Vector3 &operator=(const Vector2 &v2); // Declare assignment from Vector2
};

// Define the assignment operators after both structs
Vector2 &Vector2::operator=(const Vector3 &v3) {
  x = v3.x;
  y = v3.y;
  return *this;
}

Vector3 &Vector3::operator=(const Vector2 &v2) {
  x = v2.x;
  y = v2.y;
  z = 0;
  return *this;
}

struct Legtype {
  double coxaAngle;
  double femurAngle;
  double tibiaAngle;
  int legNumber;
  Vector2 gaitOrigin;
  Vector2 legOrigin;
  Vector3 footPosition; // Current position of the foot in 3D space
  bool isGrounded;      // Whether the foot is currently on the ground

  Legtype()
      : coxaAngle(90), femurAngle(90), tibiaAngle(90), legNumber(),
        gaitOrigin(), legOrigin(0, 0), footPosition(), isGrounded(true) {}

  // Assignment operator
  Legtype &operator=(const Legtype &other) {
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
};

/*** TEMPERARY VARIABLE DECLARATION ***/
int coxaLen = 45;       // Length of the coxa segment
int femurLen = 100;     // Length of the femur segment
int tibiaLen = 180;     // Length of the tibia segment
int bodyHeight = 50;    // Height of the chassis, used for gait calculations
Legtype legs[NUM_LEGS]; // Placeholder for Servo Data
String servoNames[3] = {"Coxa", "Femur", "Tibia"}; // Names for each servo
// const Vector2 legOrigins[6] = { // For testing!
//     Vector2(-70, 85), Vector2(-70, 0), Vector2(-70, -85),
//     Vector2(70, -85), Vector2(70, 0),  Vector2(70, 85)}; // Leg origins
// const Vector2 gaitOrigins[6] = {
//     Vector2(-150, 200), Vector2(-170, 0), Vector2(-150, -200),
//     Vector2(150, -200), Vector2(170, 0),  Vector2(150, 200)}; // Gait origins
const Vector2 legOrigins[6] = {Vector2(0, 0)};
const Vector2 gaitOrigins[6] = {Vector2(150, 0)};
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

Vector3 inverseKinematics(Legtype leg, const Vector3 &goal);
void setServoPositions(Legtype leg, Vector3 angles);
double radToDeg(double radians);

void setup() {
  Serial.begin(9600);
  Serial.println("Single Leg Test");
  // pcaDriver.begin();
  // pcaDriver.setOscillatorFrequency(26000000);
  // pcaDriver.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  for (int i = 0; i < NUM_LEGS; i++) { // Construct the leg objects
    legs[i].legNumber = i;
    legs[i].gaitOrigin = gaitOrigins[i];
    legs[i].legOrigin = legOrigins[i];
    legs[i].footPosition = gaitOrigins[i];
  }

  delay(10);

  setServoPositions(legs[0], inverseKinematics(legs[0], Vector3(200, 0, 0)));
}

void loop() {}

Vector3 inverseKinematics(Legtype leg, const Vector3 &goal) {
  // Returns a vector3 of angles given a leg object and a goal vector
  // X = coxa angle, Y = femur angle, Z = tibia angle

  const double xDistance = goal.x - leg.legOrigin.x;
  const double yDistance = goal.y - leg.legOrigin.y;
  // const double rise = bodyHeight - goal.z;

  // const double pivotToGoal =
  //     sqrt(xDistance * xDistance + yDistance * yDistance + rise * rise);

  const double theta = atan2(yDistance, xDistance);

  Vector3 effectiveGoal(goal.x - coxaLen * cos(theta),
                        goal.y - coxaLen * sin(theta), goal.z);

  // Calculate the position of the femur servo in 3d space
  // This will make checking if the goal is within reach easier
  const Vector3 femurServoPosition(
      leg.legOrigin.x + coxaLen * cos(leg.coxaAngle * M_PI / 180.0f),
      leg.legOrigin.y + coxaLen * sin(leg.coxaAngle * M_PI / 180.0f),
      bodyHeight);

  // Calculate the distance from the femur servo to the effective goal
  const double hipToGoalDistance = effectiveGoal.distanceTo(femurServoPosition);

  // Adjust goal if out of bounds
  if (hipToGoalDistance > femurLen + tibiaLen)
    effectiveGoal.lerp(femurServoPosition,
                       1 - (femurLen + tibiaLen - 0.001f) / hipToGoalDistance);

  //  accounting for the distance between the coxa and femur servos
  const Vector3 adjustedDistVect(effectiveGoal.x - femurServoPosition.x,
                                 effectiveGoal.y - femurServoPosition.y,
                                 femurServoPosition.z - effectiveGoal.z);

  const double h =
      sqrt(square(adjustedDistVect.x) + square(adjustedDistVect.y));
  const double hipToGoal = sqrt(square(h) + square(adjustedDistVect.z));

  return Vector3(
      atan2(adjustedDistVect.y, adjustedDistVect.x),
      acos((square(hipToGoal) + square(femurLen) - square(tibiaLen)) /
           (2 * hipToGoal * femurLen)) -
          atan(adjustedDistVect.z / h),
      acos((square(femurLen) + square(tibiaLen) - square(hipToGoal)) /
           (2 * femurLen * tibiaLen)));
}

void setServoPositions(Legtype leg, Vector3 angles) {
  // This function sets the servo positions for the given leg object
  Serial.println("\n\n\nGoal position: (200, 0, 0)");
  Serial.println("Leg Origin: " + String(leg.legOrigin.x) + ", " +
                 String(leg.legOrigin.y));
  Serial.println("Raw Angles (Rad) : " + String(angles.x) + ", " +
                 String(angles.y) + ", " + String(angles.z));
  Serial.println("Raw Angles (Deg) : " + String(radToDeg(angles.x)) + ", " +
                 String(radToDeg(angles.y), 5) + ", " +
                 String(radToDeg(angles.z)));
  Serial.println(
      "PWM Signal: " + String(map(angles.x, -M_PI, M_PI, SERVOMIN, SERVOMAX)) +
      ", " + String(map(angles.y, -M_PI, M_PI, SERVOMIN, SERVOMAX)) + ", " +
      String(map(angles.z, -M_PI, M_PI, SERVOMIN, SERVOMAX)));

  // pcaDriver.setPWM(leg.legNumber * 3, 0,
  //                  map(angles.x, 0, 180, SERVOMIN, SERVOMAX)); // Coxa
  // pcaDriver.setPWM(leg.legNumber * 3 + 1, 0,
  //                  map(angles.y, 0, 180, SERVOMIN, SERVOMAX)); // Femur
  // pcaDriver.setPWM(leg.legNumber * 3 + 2, 0,
  //                  map(angles.z, 0, 180, SERVOMIN, SERVOMAX)); // Tibia
}

double radToDeg(double radians) {
  // Converts radians to degrees
  return radians * double(180.0f / M_PI);
}