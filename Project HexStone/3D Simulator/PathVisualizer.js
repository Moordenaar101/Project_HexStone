// Path visualizer logic
import * as THREE from "three";

export class PathVisualizer {
	// When given an object, this class will visualize the path of the object
	// as it moves through space with a line.
	constructor(object, scene, maxPoints = 1000, color = 0xfff0000) {
		this.object = object;
		this.scene = scene;
		this.maxPoints = maxPoints;
		this.color = color;
		this.positions = new Float32Array(this.maxPoints * 3);
		this.index = 0;

		const geometry = new THREE.BufferGeometry();
		geometry.setAttribute(
			"position",
			new THREE.BufferAttribute(this.positions, 3)
		);

		const material = new THREE.LineBasicMaterial({
			color: color,
		});

		this.line = new THREE.Line(geometry, material);

		// Start the line at the object's current position
		this.reset();

		this.scene.add(this.line);
	}

	// Update the line positions.
	update() {
		if (this.line.visible) {
			const position = new THREE.Vector3();
			this.object.getWorldPosition(position);

			// Shift positions to the left.
			for (let i = 0; i < this.maxPoints - 1; i++) {
				this.positions[i * 3] = this.positions[(i + 1) * 3];
				this.positions[i * 3 + 1] = this.positions[(i + 1) * 3 + 1];
				this.positions[i * 3 + 2] = this.positions[(i + 1) * 3 + 2];
			}

			// Set the last position to the current position.
			this.positions[(this.maxPoints - 1) * 3] = position.x;
			this.positions[(this.maxPoints - 1) * 3 + 1] = position.y;
			this.positions[(this.maxPoints - 1) * 3 + 2] = position.z;

			this.line.geometry.attributes.position.needsUpdate = true;
		}
	}

	changeObject(object) {
		this.object = object;
		this.reset();
	}

	hide() {
		//Hides the line from view.
		this.line.visible = false;
	}

	show() {
		this.line.visible = true;
		this.reset();
	}

	reset() {
		for (let i = 0; i < this.maxPoints; i++) {
			const worldPosition = new THREE.Vector3();
			this.object.getWorldPosition(worldPosition);
			this.positions[i * 3] = worldPosition.x;
			this.positions[i * 3 + 1] = worldPosition.y;
			this.positions[i * 3 + 2] = worldPosition.z;
		}
	}
}
