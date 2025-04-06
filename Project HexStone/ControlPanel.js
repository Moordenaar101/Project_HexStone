// Control panel setup
import { GUI } from "three/addons/libs/lil-gui.module.min.js";

export function setupControlPannel(
	legGroups,
	legParts,
	currentGait,
	footVisualizer
) {
	const panel = new GUI({ width: 310 });

	const folder1 = panel.addFolder("Debug");
	const folder2 = panel.addFolder("Motion");
	const folder3 = panel.addFolder("Size Parameters");

	const settings = {
		Model: true,
		"Stability Polygon": true,
		"Animate Model": true,
		"Debug Object": false,
		"Debug Object Position": 0,
		"Path Visualizer": true,
		"Path Visualizer Position": 0,
		"Chassis Height": 3.5,
		"Step Length": 5,
		"Step Height": 4,
		"Straif Angle": 0,
		"Turn Threshold": 0,
		"Joint Diameter": 0.5,
		"Leg Diameter": 0.25,
		"Endpoint Diameter": 0.1,
	};

	folder1
		.add(settings, "Model")
		.listen()
		.onChange((bool) => {
			for (let i = 0; i < 6; i++) {
				legGroups[i][2].visible = bool;
			}
		});
	folder1
		.add(settings, "Stability Polygon")
		.listen()
		.onChange((bool) => {
			bool
				? (currentGait.stabMesh.material.opacity = 0.35)
				: (currentGait.stabMesh.material.opacity = 0);
		});
	folder1
		.add(settings, "Debug Object")
		.listen()
		.onChange((bool) => {
			bool
				? (currentGait.debugShape.material.opacity = 1)
				: (currentGait.debugShape.material.opacity = 0);
		});
	folder1
		.add(settings, "Debug Object Position", 0, 5, 1)
		.listen()
		.onChange((val) => {
			currentGait.debugShapePos = val;
		});
	folder1
		.add(settings, "Path Visualizer")
		.listen()
		.onChange((val) => {
			val == false ? footVisualizer.hide() : footVisualizer.show();
		});
	folder1
		.add(settings, "Path Visualizer Position", 0, 5, 1)
		.listen()
		.onChange((val) => {
			footVisualizer.changeObject(legParts[val][5]);
		});
	folder2
		.add(settings, "Animate Model")
		.listen()
		.onChange((bool) => {
			animateModel = bool;
		});
	folder2
		.add(settings, "Chassis Height", 0, 10, 0.1)
		.listen()
		.onChange((val) => {
			chassisHeight = val;
			for (let i = 0; i < 6; i++) {
				legGroups[i][2].position.y = chassisHeight;
			}
		});
	folder2
		.add(settings, "Step Length", 2, 8, 0.1)
		.listen()
		.onChange((val) => {
			rippleGait.stepDis = val;
		});
	folder2
		.add(settings, "Step Height", 0, 8, 0.1)
		.listen()
		.onChange((val) => {
			rippleGait.stepHeight = val;
		});
	folder2
		.add(settings, "Straif Angle", -90, 90, 1)
		.listen()
		.onChange((val) => {
			rippleGait.straifAngle = MathUtils.degToRad(val);
		});
	folder2
		.add(settings, "Turn Threshold", -100, 100, 1)
		.listen()
		.onChange((val) => {
			rippleGait.turnPoint.x = val;
			rippleGait.turnThreshold = val;
			centerPoint.position.x = val;
		});
	folder3
		.add(settings, "Joint Diameter", 0.1, 5, 0.1)
		.listen()
		.onChange((val) => {
			for (let i = 0; i < 6; i++) {
				legParts[i][0].scale.set(val * 1.5, 1.25, val * 1.5);
				legParts[i][1].scale.set(val, val, 1);
				legParts[i][3].scale.set(val, val, 1);
				legParts[i][5].scale.set(val, val, 1);
			}
		});
	folder3
		.add(settings, "Leg Diameter", 0.1, 5, 0.1)
		.listen()
		.onChange((val) => {
			for (let i = 0; i < 6; i++) {
				legParts[i][2].scale.set(val, val);
				legParts[i][4].scale.set(val, val);
			}
		});

	folder1.open();
	folder2.open();
}
