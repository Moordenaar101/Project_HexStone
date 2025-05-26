// Leg generation logic
import * as THREE from "three";

export function generateLegs(
	scene,
	hipStance,
	chassisHeight,
	coxaLen,
	femurLen,
	tibiaLen,
	hipDis
) {
	// Legs
	/*
							 Back
			           (5)         (0)
			             \  _____  /
			              \/     \/
			         (4)___|     |___(1)
			               |     |
			              /\__↓__/\
			             /         \
			           (3)         (2)
							Front
				*/
	// Parts: ***Need to add coxa***
	// 0 = pivot
	// 1 = hip
	// 2 = femur
	// 3 = knee
	// 4 = tibia
	// 5 = foot

	// Groups:
	// legGroup[i][0] = (Tibia, Foot)
	// legGroup[i][1] = (Hip, Femur, Knee, legGroup[i][0])
	// legGroup[i][2] = (Pivot, legGroup[i][1])

	let legGroups = [];
	let legParts = [];

	const jointNames = ["Pivot", "Hip", "Femur", "Knee", "Tibia", "Foot"];
	const legGeometrySpecs = [
		{ color: "yellow", rotate: false }, // Pivot
		{ color: "red", rotate: true }, // Hip
		{ color: "purple", rotate: true }, // Femur
		{ color: "blue", rotate: true }, // Knee
		{ color: "purple", rotate: true }, // Tibia
		{ color: "blue", rotate: true }, // Foot
	];
	const legPositions = [
		{ x: hipStance[0][0] / 2, z: hipStance[0][1] },
		{ x: hipStance[1][0] / 2, z: hipStance[1][1] },
		{ x: hipStance[2][0] / 2, z: hipStance[2][1] },
		{ x: -hipStance[2][0] / 2, z: hipStance[2][1] },
		{ x: -hipStance[1][0] / 2, z: hipStance[1][1] },
		{ x: -hipStance[0][0] / 2, z: hipStance[0][1] },
	];

	const createJoint = (spec) => {
		const geometry = new THREE.CylinderGeometry(1, 1, 1, 32);
		if (spec.rotate) geometry.rotateX(Math.PI * 0.5);
		const material = new THREE.MeshPhongMaterial({ color: spec.color });
		return new THREE.Mesh(geometry, material);
	};

	const createGroup = (name) => {
		const group = new THREE.Group();
		group.name = name;
		return group;
	};

	const setLegScale = (leg) => {
		const scales = [
			[0.75, 1.25, 0.75], // Pivot
			[0.5, 0.5, 1], // Hip
			[0.25, 0.25, 5], // Femur
			[0.5, 0.5, 1], // Knee
			[0.25, 0.25, 5], // Tibia
			[0.5, 0.5, 1], // Foot
		];
		scales.forEach((scale, index) => leg[index].scale.set(...scale));
	};

	const createLabel = (text) => {
		const canvas = document.createElement("canvas");
		const context = canvas.getContext("2d");
		context.font = "48px Arial";
		context.fillStyle = "white";
		context.fillText(text, 0, 50);

		const texture = new THREE.CanvasTexture(canvas);
		const material = new THREE.SpriteMaterial({ map: texture });
		const sprite = new THREE.Sprite(material);
		sprite.scale.set(5, 2.5, 1);
		return sprite;
	};

	for (let i = 0; i < 6; i++) {
		const objectArray = legGeometrySpecs.map((spec, index) => {
			const joint = createJoint(spec);
			joint.name = `${jointNames[index]} | Leg ${i}`;
			return joint;
		});

		legParts.push(objectArray);
		setLegScale(objectArray);

		const [pivot, hip, femur, knee, tibia, foot] = objectArray;

		const group0 = createGroup(`Group 0 | Leg ${i}`);
		group0.add(tibia, foot);
		group0.position.x = femurLen;

		const group1 = createGroup(`Group 1 | Leg ${i}`);
		group1.add(hip, femur, knee, group0);
		group1.position.y += hipDis;
		group1.position.x += coxaLen;

		const group2 = createGroup(`Group 2 | Leg ${i}`);
		group2.add(pivot, group1);

		// Add label for each leg
		const label = createLabel(`#${i}`);
		label.position.set(0, 2, 0);
		group2.add(label);

		legGroups.push([group0, group1, group2]);

		femur.position.x = femurLen / 2;
		knee.position.x = tibiaLen;
		tibia.position.x = tibiaLen / 2;
		foot.position.x = tibiaLen;
		let temp = new THREE.Vector3();
		knee.getWorldPosition(temp);
		femur.lookAt(temp);
		foot.getWorldPosition(temp);
		tibia.lookAt(temp);

		if (i > 2) group2.rotation.y += Math.PI;

		scene.add(group2);

		group2.position.set(legPositions[i].x, chassisHeight, legPositions[i].z);
	}

	return { legGroups, legParts };
}
