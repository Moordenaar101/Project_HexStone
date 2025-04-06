// Scene setup functions
import * as THREE from "three";
import { OrbitControls } from "ImportControls";

export let camera, scene, renderer;

export function init() {
	console.clear();

	camera = new THREE.PerspectiveCamera(
		75,
		window.innerWidth / window.innerHeight,
		0.1,
		1000
	);
	camera.position.set(35, 25, 30);

	scene = new THREE.Scene();

	renderer = new THREE.WebGLRenderer({ antialias: true });
	renderer.setPixelRatio(window.devicePixelRatio);
	renderer.setClearColor("#2c354d");
	renderer.setSize(window.innerWidth, window.innerHeight);
	document.body.appendChild(renderer.domElement);

	const controls = new OrbitControls(camera, renderer.domElement);
	window.addEventListener("resize", onWindowResize);

	setupScene();

	controls.target = new THREE.Vector3(0, 10, 0);
	controls.update();
}

function onWindowResize() {
	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();
	renderer.setSize(window.innerWidth, window.innerHeight);
}

function setupScene() {
	const light1 = new THREE.AmbientLight({ color: "#ffffff" });
	const light2 = new THREE.DirectionalLight({ color: "#ffffff" }, 5);
	light2.position.set(3, 9, 0);

	scene.add(light1);
	scene.add(light2);

	const size = 100;
	const divisions = 100;
	const gridColor = new THREE.Color("rgb(75, 75, 75)");
	const centerColor = new THREE.Color("rgb(75, 25, 50)");

	const gridHelper = new THREE.GridHelper(
		size,
		divisions,
		centerColor,
		gridColor
	);
	scene.add(gridHelper);

	const axesHelper = new THREE.AxesHelper(5);
	axesHelper.position.y += 0.005;
	scene.add(axesHelper);
}
