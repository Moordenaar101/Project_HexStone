// Gait logic
import * as THREE from "three";
// import { scene } from "./SceneSetup.js";
import {
	gaitBDC,
	buildGoals,
	mapVal,
	DebuggingObject,
} from "./HexapodUtils.js";

let gaitCenterPoints = [],
	gaitStartPoints = [],
	gaitEndPoints = [];

export class Gait {
	constructor(name, offsets, maxSpeed, stepDis, stepHeight, speed) {
		this.name = name;
		this.offsets = offsets;
		this.maxSpeed = maxSpeed;
		this.onGround = Array(6).fill(true);
		this.centerPoint = new THREE.Vector3();

		/*** Movement Parameters ***/
		this.stepDis = stepDis;
		this.stepHeight = stepHeight;
		this.straifAngle = 0;
		this.turnThreshold = 0;
		// this.turnRadius = 0;
		this.turnPoint = new THREE.Vector3();
		this.speed = speed;
	}

	updateWalkCycle(legGroups, legParts, points, goals) {
		const gaitRatio = 1 - 4 / 6;

		// function calculateGaitCenterPoint(control, legID) {
		// 	const legPosition = legParts[legID][0].getWorldPosition(
		// 		new THREE.Vector3()
		// 	);
		// 	return new THREE.Vector3(
		// 		(control.x + Math.abs(legPosition.x)) * Math.sign(legPosition.x),
		// 		0,
		// 		legPosition.z
		// 	);
		// }

		if (
			this.turnThreshold > 99 ||
			this.turnThreshold < -99 ||
			this.turnThreshold == 0
		) {
			this.turnPoint = new THREE.Vector3(100, 0, 0);
		}

		// console.log(legGroups);

		for (let i = 0; i < 6; i++) {
			gaitCenterPoints[i] = new THREE.Object3D().position.set(
				gaitBDC(i, legGroups).x,
				gaitBDC(i, legGroups).y,
				gaitBDC(i, legGroups).z
			);

			gaitStartPoints[i] = new THREE.Object3D().position.set(
				gaitBDC(i, legGroups).x,
				gaitBDC(i, legGroups).y,
				gaitBDC(i, legGroups).z + 5 / 2
			);

			gaitEndPoints[i] = new THREE.Object3D().position.set(
				gaitBDC(i, legGroups).x,
				gaitBDC(i, legGroups).y,
				gaitBDC(i, legGroups).z - 5 / 2
			);
		}

		legGroups.forEach((_, i) => {
			if (this.offsets[i] > 1000) this.offsets[i] = 0;

			const straif = this.straifAngle;

			// Remember the original start and end position before offsetting.
			let startOrigin = gaitCenterPoints[i];
			startOrigin.y = 0;
			let endOrigin = gaitCenterPoints[i];
			endOrigin.y = 0;

			// Recalculate positions of start and end points
			// based on the straif angle.

			// gaitStartPoints[i].x =
			// 	Math.cos(straif) * (startOrigin.x - gaitCenterPoints[i].x) +
			// 	Math.sin(straif) * (startOrigin.z - 2.5 - gaitCenterPoints[i].z) +
			// 	gaitCenterPoints[i].x;

			// gaitStartPoints[i].z =
			// 	Math.sin(straif) * (startOrigin.x - gaitCenterPoints[i].x) -
			// 	Math.cos(straif) * (startOrigin.z - 2.5 - gaitCenterPoints[i].z) +
			// 	gaitCenterPoints[i].z;

			// gaitEndPoints[i].x =
			// 	Math.cos(straif) * (endOrigin.x - gaitCenterPoints[i].x) +
			// 	Math.sin(straif) * (endOrigin.z + 2.5 - gaitCenterPoints[i].z) +
			// 	gaitCenterPoints[i].x;
			// gaitEndPoints[i].z =
			// 	Math.sin(straif) * (endOrigin.x - gaitCenterPoints[i].x) -
			// 	Math.cos(straif) * (endOrigin.z + 2.5 - gaitCenterPoints[i].z) +
			// 	gaitCenterPoints[i].z;

			function adjustPointsToArc(
				start,
				center,
				end,
				arcCenter,
				radius,
				arcLength
			) {
				// Calculate the angle of the start and end point relative to the arc center
				const offsetAngle = arcLength / 2 / radius;
				const centerAngle = Math.atan2(
					center.z - arcCenter.z,
					center.x - arcCenter.x
				);
				// console.log(offsetAngle, centerAngle);

				// Calculate the new start and end points on the arc
				const newStart = new THREE.Vector3(
					arcCenter.x + radius * Math.cos(centerAngle + offsetAngle),
					start.y, // Preserve the original Y-coordinate
					arcCenter.z + radius * Math.sin(centerAngle + offsetAngle)
				);

				const newEnd = new THREE.Vector3(
					arcCenter.x + radius * Math.cos(centerAngle - offsetAngle),
					end.y, // Preserve the original Y-coordinate
					arcCenter.z + radius * Math.sin(centerAngle - offsetAngle)
				);

				return { newStart, newEnd };
			}

			// Testing the arc adjustment function
			const arcCenter = new THREE.Vector3(this.turnThreshold, 0, 0); // Define the arc center
			const radius = gaitCenterPoints[0].distanceTo(arcCenter); // Calculate the radius

			// const { newStart, newEnd } = adjustPointsToArc(
			// 	gaitStartPoints[0],
			// 	gaitCenterPoints[0],
			// 	gaitEndPoints[0],
			// 	arcCenter,
			// 	radius,
			// 	this.stepDis
			// );
			// newgaitStartPoints[i] = newStart;
			// newgaitEndPoints[i] = newEnd;

			let calcedRatio = this.offsets[i] / (points / 2);
			// calcedRatio = 1;
			const isGrounded = calcedRatio > 2 * gaitRatio;
			let calcedOffset = isGrounded
				? mapVal(calcedRatio, 2 * gaitRatio, 2, 1, 2)
				: mapVal(calcedRatio, 0, 2 * gaitRatio, 0, 1);
			// console.log(calcedOffset);
			const calcPointOnGround = (p1, p2, p3) =>
				(calcedOffset - 1) * (calcedOffset - 1) * p1 +
				2 * (calcedOffset - 1) * (1 - (calcedOffset - 1)) * p2 +
				(1 - (calcedOffset - 1)) * (1 - (calcedOffset - 1)) * p3;
			const calcPoint = (p1, p2, p3) =>
				(1 - calcedOffset) * (1 - calcedOffset) * p1 +
				2 * (1 - calcedOffset) * calcedOffset * p2 +
				calcedOffset * calcedOffset * p3;
			const calcPointY = () =>
				2 * (1 - calcedOffset) * calcedOffset * (this.stepHeight * 2);

			let calcX = isGrounded
				? calcPointOnGround(
						gaitStartPoints[i].x,
						gaitCenterPoints[i].x,
						gaitEndPoints[i].x
				  )
				: calcPoint(
						gaitStartPoints[i].x,
						gaitCenterPoints[i].x,
						gaitEndPoints[i].x
				  );

			let calcZ = isGrounded
				? calcPointOnGround(
						gaitStartPoints[i].z,
						gaitCenterPoints[i].z,
						gaitEndPoints[i].z
				  )
				: calcPoint(
						gaitStartPoints[i].z,
						gaitCenterPoints[i].z,
						gaitEndPoints[i].z
				  );

			const calcY = isGrounded ? 0 : calcPointY();

			goals[i].set(calcX, calcY, calcZ);
			legParts[i][5].material.color.set(isGrounded ? "blue" : "red");

			this.offsets[i] += 5;
			this.onGround[i] = isGrounded;
		});

		// this.updateDebugShape();
		return goals;
	}
}
