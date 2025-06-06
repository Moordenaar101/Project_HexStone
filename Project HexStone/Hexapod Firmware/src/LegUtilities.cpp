#include "utilities.h"
#include <Arduino.h>

// Example: You may need to define these based on your robot's geometry
extern const float coxaLen;
extern const float femurLen;
extern const float tibiaLen;
extern const Vector3 legOrigin; // The base position of the leg in 3D space

// // Optionally, add a norm() method to Vector3 for vector length
// inline float vector3Norm(const Vector3 &v) {
//   return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
// }

// Returns true if the goal is within the leg's reachable workspace
inline bool legReachable(ServoData legs, const Vector3 &goal) {
  // Calculate the distance from the leg origin to the goal
  float distance = goal.distanceTo(legOrigin);

  // The leg can reach if the distance is less than or equal to the sum of the
  // segments
  float maxReach = femurLen + tibiaLen;
  return distance <= maxReach;
}

Vector3 shiftGoal(ServoData legs, Vector3 goal) {
  // This function shifts the goal position to ensure it is within the leg's
  // reachable workspace.
  // For simplicity, we will just return the leg origin if the goal is not
  // reachable. In a real implementation, you might want to adjust the goal
  // position based on the leg's geometry.

  if (!legReachable(legs, goal)) {
    return goal.lerp(legOrigin,
                     (1 - (femurLen + tibiaLen - 0.001)) /
                         goal.distanceTo(legOrigin)); // Shift goal towards the
  }
  return goal; // Return the original goal if reachable
}

void calcServoAngles(ServoData leg, Vector3 goal) {
  // This function calculates the angles for the servos based on the desired
  // position (goal).
  shiftGoal(leg, goal);

  float xDist = goal.x - legOrigin.x;
  float yDist = goal.y - legOrigin.y;
  float zDist = goal.z - legOrigin.z;

  float hypotenuse = sqrt(xDist * xDist + yDist * yDist + zDist * zDist);

  float theta = atan2(yDist, xDist);

  const Vector3 effectiveGoal(goal.x - coxaLen * cos(theta),
                              goal.y - coxaLen * sin(theta), goal.z);

  const float hipToGoalDistance = effectiveGoal.distanceTo(legOrigin);
}

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