#include <Arduino.h>

void calcServoAngles(int goal) {
  // This function calculates the angles for the servos based on the desired
  // position (goal).
  /*
  legReachable(goal)?
          0 :
          goal = shiftGoal(goal);

  float xDist = goal.x - legOrigin.x;
  float yDist = goal.y - legOrigin.y;
  float zDist = goal.z - legOrigin.z;

  hypotenuse = sqrt(xDist * xDist + yDist * yDist + zDist * zDist);

  float theta = atan2(yDist, xDist);

  const Vector3 effectiveGoal = (goal.x - coxaLength * cos(theta),
  goal.y - coxaLength * sin(theta),
  goal.z)

  */
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