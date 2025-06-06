#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

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
  String servoName;   // <-- Added variable for servo name
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

/*** TEMPERARY VARIABLE DECLARATION ***/
float coxaLen = 0.0;   // Length of the coxa segment
float femurLen = 0.0;  // Length of the femur segment
float tibiaLen = 0.0;  // Length of the tibia segment
Vector3 legOrigins[6]; // Base position of the leg in 3D space
ServoData legs[6][3];  // Placeholder for Servo Data
String servoNames[3] = {"Coxa", "Femur", "Tibia"}; // Names for each servo
/**************************************/

// void driveServos(legtype leg, float goalAngles)

//     void setup1() {
//   Serial.begin(9600);
//   Serial.println("Single Leg Test");

//   /*
//    * In theory the internal oscillator (clock) is 25MHz but it really isn't
//    * that precise. You can 'calibrate' this by tweaking this number until
//    * you get the PWM update frequency you're expecting!
//    * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
//    * is used for calculating things like writeMicroseconds()
//    * Analog servos run at ~50 Hz updates, It is importaint to use an
//    * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
//    * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
//    *    the I2C PCA9685 chip you are setting the value for.
//    * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
//    *    expected value (50Hz for most ESCs)
//    * Setting the value here is specific to each individual I2C PCA9685 chip
//    and
//    * affects the calculations for the PWM update frequency.
//    * Failure to correctly set the int.osc value will cause unexpected PWM
//    * results
//    */

//   pcaDriver.begin();
//   pcaDriver.setOscillatorFrequency(26000000);
//   pcaDriver.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

//   for (int i = 0; i < 6; i++) {
//     for (int j = 0; j < 3; j++) {
//       legs[i][j] =
//           ServoData(0, "|> Leg " + String(i) + " | " + servoNames[j] + " <|",
//                     0.0, SERVOMIN, SERVOMAX / 2);
//     }
//   }
//   delay(10);
// }

void loop1() {
  int servoNum = 0;
  int pulseLen = (SERVOMAX / SERVOMIN) / 2; // Start at mid-range
  pcaDriver.setPWM(servoNum, 0, pulseLen);

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pcaDriver.setPWM(servoNum, 0, pulselen);
  }
}

// void calcServoAngles(Vector3 goal) {
//   // This function calculates the angles for the servos based on the desired
//   // position (goal).
//   shiftGoal(legs, goal);

//   float xDist = goal.x - legOrigin.x;
//   float yDist = goal.y - legOrigin.y;
//   float zDist = goal.z - legOrigin.z;

//   float hypotenuse = sqrt(xDist * xDist + yDist * yDist + zDist * zDist);

//   float theta = atan2(yDist, xDist);

//   const Vector3 effectiveGoal(goal.x - coxaLen * cos(theta),
//                               goal.y - coxaLen * sin(theta), goal.z);

//   const float hipToGoalDistance = effectiveGoal.distanceTo(legOrigin);
// }

/*
export function calcAngles(goals, legParts, coxaLen, femurLen, tibiaLen) {
        const jointAngles = goals.map((goal, i) => {
                const pivotPosition = legParts[i][0].getWorldPosition(new
THREE.Vector3()); const hipPosition = legParts[i][1].getWorldPosition(new
THREE.Vector3());

                // Calculate the distance from the pivot to the goal
                const xDistance = goal.x - pivotPosition.x;
                const zDistance = goal.z - pivotPosition.z;
                const rise = pivotPosition.y - goal.y;
                const pivotToGoal = Math.sqrt(xDistance ** 2 + zDistance ** 2 +
rise ** 2);

                // Calculate the angle of the coxa rotation
                const theta = Math.atan2(zDistance, xDistance);

                // Calculate the effective distance from the hip to the goal
                const effectiveGoalX = goal.x - coxaLen * Math.cos(theta);
                const effectiveGoalZ = goal.z - coxaLen * Math.sin(theta);
                const effectiveGoal = new THREE.Vector3(
                        effectiveGoalX,
                        goal.y,
                        effectiveGoalZ
                );
                const hipToGoalDistance = effectiveGoal.distanceTo(hipPosition);

                // Adjust goal if out of bounds
                if (hipToGoalDistance > femurLen + tibiaLen) {
                        effectiveGoal.lerp(
                                hipPosition,
                                1 - (femurLen + tibiaLen - 0.001) /
hipToGoalDistance
                        );
                }

                // Calculate distances and angles
                const adjustedXDistance = effectiveGoal.x - hipPosition.x;
                const adjustedZDistance = effectiveGoal.z - hipPosition.z;
                const adjustedRise = hipPosition.y - effectiveGoal.y;
                const h = Math.sqrt(adjustedXDistance ** 2 + adjustedZDistance
** 2); const hipToGoal = Math.sqrt(h ** 2 + adjustedRise ** 2);

                const joint1 = Math.atan2(adjustedZDistance, adjustedXDistance);
// Pivot joint angle const angleA = Math.atan(adjustedRise / h); const angleB =
Math.acos( (hipToGoal ** 2 + femurLen ** 2 - tibiaLen ** 2) / (2 * hipToGoal *
femurLen)
                );
                const joint2 = angleB - angleA; // Hip joint angle
                const joint3 = Math.acos(
                        (femurLen ** 2 + tibiaLen ** 2 - hipToGoal ** 2) /
                                (2 * femurLen * tibiaLen)
                ); // Knee joint angle

                return [joint1, joint2, joint3];
        });

        return jointAngles;
}
*/

// void setServoPositions(int leg, Vector3 angles) {
//   // This function sets the servo positions based on the calculated angles.
//   // The angles are in degrees and should be converted to the appropriate
//   // pulse width for the servos.

//   // Example of setting servo positions:
//   // pcaDriver.setPWM(leg * 3 + 0, 0, angleToPulseWidth(angles.x));
//   // pcaDriver.setPWM(leg * 3 + 1, 0, angleToPulseWidth(angles.y));
//   // pcaDriver.setPWM(leg * 3 + 2, 0, angleToPulseWidth(angles.z));
// }