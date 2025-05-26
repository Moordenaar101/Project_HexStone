import * as THREE from "three";
import { init, scene } from "./SceneSetup.js";

export function gaitBDC(index, legGroups, gaitOffset = [6, 0, 0]) {
	// Returns the offset Position of the gait BDC (Bottom Dead Center) for
	// a given leg index.
	// index = leg #, gaitOffset = offset from the leg pivot to the gait
	// center point.
	// console.log(legGroups)
	let indexPos = legGroups[index][2].getWorldPosition(new THREE.Vector3());

	let gaitCenterPoint =
		index < 3
			? new THREE.Vector3(
					indexPos.x + gaitOffset[0],
					0,
					indexPos.z + gaitOffset[2]
			  )
			: new THREE.Vector3(
					indexPos.x - gaitOffset[0],
					0,
					indexPos.z + gaitOffset[2]
			  );
	return gaitCenterPoint;
}

export function buildGoals(legGroups) {
	// Builds and sets all goals to their default positions all at once
	let goals = [];
	for (let i = 0; i < legGroups.length; i++) {
		goals.push(gaitBDC(i, legGroups));
	}
	return goals;
}

export function mapVal(value, fromMin, fromMax, toMin, toMax) {
	return ((value - fromMin) * (toMax - toMin)) / (fromMax - fromMin) + toMin;
}

// export function initDebugShape() {
// 	const debugShape = new THREE.Mesh(
// 		new THREE.SphereGeometry(0.75, 32, 32),
// 		new THREE.MeshBasicMaterial({
// 			color: 0xffff00,
// 			transparent: true,
// 			opacity: 0,
// 			name: "Debug Shape",
// 		})
// 	);
// 	scene.add(debugShape);
// 	return debugShape;
// }

export class DebuggingObject {
	constructor(target, radius = 0.5, color = 0xffff00) {
		this.target = target;
		this.radius = radius;
		this.color = color;
		this.object = new THREE.Mesh(
			new THREE.SphereGeometry(this.radius, 32, 32),
			new THREE.MeshBasicMaterial({
				color: this.color,
				transparent: true,
				opacity: 0,
				name: "Debugging Object",
			})
		);
		this.object.position.copy(this.target.position);
		scene.add(this.object);
	}

	updatePositon(target) {
		this.object.position.copy(target.position);
	}

	hide() {
		this.object.material.opacity = 0;
	}
}

export class stabilityMesh {
	constructor() {
		this.positionArray;
	}
	init(legParts, color = 0x990000) {
		this.color = color;
		this.positionArray = [];
		legParts.forEach((object) => {
			positionArray.push(
				new THREE.Vector2(
					object[5].getWorldPosition(new THREE.Vector3()).x,
					object[5].getWorldPosition(new THREE.Vector3()).z
				)
			);
		});
		this.mesh = new THREE.Mesh(
			new THREE.ShapeGeometry(new THREE.Shape(this.positionArray)),
			new THREE.MeshBasicMaterial({
				color: this.color,
				transparent: true,
				opacity: 0.35,
				side: THREE.DoubleSide,
			})
		);
		this.mesh.rotateX(Math.PI / 2);
		this.mesh.name = "Stability Mesh";
	}
	updateMesh(legParts) {
		if (this.positionArray == undefined) {
			init(legParts);
		} else {
			const vectArray = legParts.map((leg) => {
				return new THREE.Vector2(
					leg[5].getWorldPosition(new THREE.Vector3()).x,
					leg[5].getWorldPosition(new THREE.Vector3()).z
				);
			});

			const f32Array = new Float32Array(
				vectArray.flatMap(({ x, y }) => [x, y, 0])
			);
			this.mesh.geometry.attributes.position.array = f32Array;
			this.mesh.geometry.attributes.position.needsUpdate = true;

			const tempPos = new THREE.Vector3();
			const pointA = new THREE.Vector2();
			const pointB = new THREE.Vector2();

			for (let i = 0; i < 6; i++) {
				if (legParts[i][5].position.y == 0) {
					const downIndex = findAdjacentGroundLeg(i, -1);
					const upIndex = findAdjacentGroundLeg(i, 1);

					legParts[downIndex][5].getWorldPosition(tempPos);
					pointA.set(tempPos.x, tempPos.z);

					legParts[upIndex][5].getWorldPosition(tempPos);
					pointB.set(tempPos.x, tempPos.z);

					vectArray[i].lerpVectors(pointA, pointB, 0.5);
				}
			}
		}
		function findAdjacentGroundLeg(legParts, index, direction) {
			let offset = 0;
			while (legParts[(index + direction * offset + 6) % 6][5].position.y > 0) {
				offset++;
			}
			return (index + direction * offset + 6) % 6;
		}
	}

	// 	checkStability() {
	// 		if (this.positionArray == undefined) {
	// 			init(legParts);
	// 		} else {
	// 			let onGroundCount = 0;
	// 			for (let i = 0; i < 6; i++) {
	// 				legParts[i][5].getWorldPosition(new THREE.Vector3()) == 0
	// 					? onGroundCount++
	// 					: (onGroundCount += 0);
	// 			}
	// 			if (onGroundCount < 3) console.warn("Less than 3 points of contact!");

	// 			this.onGround.forEach((isGrounded, i) => {
	// 				if (isGrounded) {
	// 					const worldPos = new THREE.Vector3();
	// 					legParts[i][5].getWorldPosition(worldPos);
	// 					this.stabilityPolygon.push(worldPos.x, 0, worldPos.z);
	// 				}
	// 			});

	// 			this.isStable();
	// 		}
	// 	}

	// 	isStable() {
	// 		const numPoints = this.mesh.length;
	// 		let p1 = this.stabilityPolygon[0];
	// 		let stable = false;

	// 		for (let i = 1; i <= numPoints; i++) {
	// 			const p2 = this.stabilityPolygon[i % numPoints];
	// 			if (
	// 				this.centerPoint.z > Math.min(p1[1], p2[1]) &&
	// 				this.centerPoint.z <= Math.max(p1[1], p2[1]) &&
	// 				this.centerPoint.x <= Math.max(p1[0], p2[0])
	// 			) {
	// 				const xIntersection =
	// 					((this.centerPoint.z - p1[0]) * (p2[1] - p1[1])) / (p2[0] - p1[0]) +
	// 					p1[1];
	// 				stable = p1[1] === p2[1] || this.centerPoint.x <= xIntersection;
	// 			}
	// 			p1 = p2;
	// 		}

	// 		this.stable = stable;
	// 	}
}
