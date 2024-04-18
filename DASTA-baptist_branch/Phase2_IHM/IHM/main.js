import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls"


const canvas = document.querySelector(".canvas")
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, canvas.offsetWidth / canvas.offsetHeight, 0.1, 100);
camera.position.set(0, 4,  0)
// Update the camera's projection matrix after making changes
camera.updateProjectionMatrix();
const voltage_1 = document.querySelector(".vol-1")
const voltage_2 = document.querySelector(".vol-2")
const voltage_3 = document.querySelector(".vol-3")

// const flight_pageBtn = document.querySelector(".flight-control")
// const settings_pageBtn = document.querySelector(".settings")
// const planningBtn = document.querySelector(".planning")

// const flight_page = document.querySelector(".flight-page")
// const settings_page = document.querySelector(".settings-page")
// const planning = document.querySelector(".planning-page")

// flight_pageBtn.addEventListener("click", (event) => {
  //   settings_page.style.display = "none"
  //   planning.style.display = "none"
  //   flight_page.style.display = "block"
  // })
  // settings_pageBtn.addEventListener("click", (event) => {
    //   settings_page.style.display = "block"
    //   planning.style.display = "none"
    //   flight_page.style.display = "none"
    // })
    // planningBtn.addEventListener("click", (event) => {
//   settings_page.style.display = "none"
//   planning.style.display = "block"
//   flight_page.style.display = "none"
// })

// let z_coord = document.querySelector(".z-coord")
// let y_coord = document.querySelector(".y-coord")
// let x_coord = document.querySelector(".x-coord")

// //Visual inputs values for the ranges
// const XY_range = document.getElementById("xy")
// const Y_range = document.getElementById("y")
// const range0 = document.getElementById("0")


//Getting the input values for the check boxes
// const data_rec = document.getElementById("data recording")
// const zone_res = document.getElementById("zone restriction")
// const position_stab = document.getElementById("position stabilization")


//Getting Buttons from DOM
// const startBtn = document.getElementById("start")
// const stopBtn = document.getElementById("stop")


// const btnLeft = document.querySelector(".left")
// const btnRight = document.querySelector(".right")


let droneModel


const renderer = new THREE.WebGLRenderer({ canvas });
renderer.shadowMap.enabled = true
//
renderer.setSize(window.innerWidth , window.innerHeight)


const geometry = new THREE.SphereGeometry(1, 64, 64);
const material = new THREE.MeshStandardMaterial({ color: 0x00ff00 });
const cube = new THREE.Mesh(geometry, material);

import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';

const objLoader = new OBJLoader();
const mtlLoader = new MTLLoader();

const initial = new THREE.Euler(0, 0, 0, 'ZYX');

var orientation_x = 0
var orientation_y = 0
var orientation_z = 0
let target_orientation_x = 0;
let target_orientation_y = 0;
let target_orientation_z = Math.PI / 2;
const dampingFactor = 0.1; // Adjust the damping factor as needed

function animate() {
  requestAnimationFrame(animate);

  // Calculate the orientation adjustments
  const deltaX = target_orientation_x - orientation_x;
  const deltaY = target_orientation_y - orientation_y;
  const deltaZ = target_orientation_z - orientation_z;

  // Apply damping to the orientation adjustments
  orientation_x += deltaX * dampingFactor;
  orientation_y += deltaY * dampingFactor;
  orientation_z += deltaZ * dampingFactor;

  // Apply the new orientation to the drone model
  if (droneModel) {
    initial.set(orientation_x, orientation_y, orientation_z, 'ZYX');
    droneModel.setRotationFromEuler(initial);
    // droneModel.position.set(orientation_x, orientation_y, orientation_z)
  }

  render();
}


scene.background = new THREE.Color(0x92898A);


mtlLoader.load(
  'DASTA v1.mtl',
  function (materials) {
    materials.preload();
    objLoader.setMaterials(materials);
    
    objLoader.load(
      'DASTA v1.obj',
      function (object) {
        scene.add(object);
        droneModel = object
        droneModel.scale.set(0.1, 0.1, 0.1)
        droneModel.setRotationFromEuler(initial)
        // Optionally manipulate the loaded object
        camera.lookAt(droneModel?.position);
      },
      function (xhr) {
        console.log((xhr.loaded / xhr.total * 100) + '% loaded');
      },
      function (error) {
        console.error('Failed to load OBJ file:', error);
      }
    );
  },
  function (xhr) {
    console.log((xhr.loaded / xhr.total * 100) + '% loaded');
  },
  function (error) {
    console.error('Failed to load MTL file:', error);
  }
);

const control = new OrbitControls(camera, canvas)

const directionLight = new THREE.DirectionalLight(0xffffff, 5)
directionLight.position.z = -5
directionLight.castShadow = true
scene.add(directionLight)



//creating a plane floor
const floor = new THREE.BoxGeometry(8, 8, 0.4);
const floorMaterial = new THREE.MeshBasicMaterial({ color: "#e6ebff", side: THREE.DoubleSide });
const plane = new THREE.Mesh(floor, floorMaterial);
plane.receiveShadow = true
plane.rotation.x = Math.PI / 2;
plane.position.y = -0.22
// scene.add(plane);

control.enableDamping = true

// let isMoving;
// startBtn.addEventListener("click", () => {
//   isMoving = true; // Set the flag when the button is clicked
// });
// stopBtn.addEventListener("click", () => {
//   isMoving = false; // Set the flag when the button is clicked
// });


// const ctx = document.getElementById('myChart');

var X_axisData = {
  label: "X axis",
  data: [],
  lineTension: true,
  fill: false,
  borderColor: 'red'
};

var Y_axisData = {
  label: "Y axis",
  data: [],
  lineTension: true,
  fill: false,
  borderColor: 'blue'
};
var Z_axisData = {
  label: "Z axis",
  data: [],
  lineTension: true,
  fill: false,
  borderColor: 'yellow'
};


var speedData = {
  labels: [],
  datasets: [X_axisData, Y_axisData, Z_axisData]
};

var chartOptions = {
  legend: {
    display: true,
    position: 'top',
    labels: {
      boxWidth: 100,
      fontColor: 'black'
    }
  }
};



// var lineChart = new Chart(ctx, {
//   type: 'line',
//   data: speedData,
//   options: chartOptions
// });



animate();

function render(){
  renderer.render(scene, camera)

}

const axesHelper = new THREE.AxesHelper( 5 );
axesHelper.scale.set(0.5, 0.5, 0.5)
scene.add( axesHelper );

axesHelper.setColors ("red", "green", "blue" )

import axios from 'axios';

async function fetchDataFromAPI(url) {
  try {
    // Set up axios instance with CORS headers
    const axiosInstance = axios.create({
      headers: {
        'Access-Control-Allow-Origin': '*', // Allow requests from any origin
        'Content-Type': 'application/json' // Set content type as JSON
      }
    });

    // Make the request using the axios instance
    const response = await axiosInstance.get(url);
    const data = response.data;

    // Return the data
    console.log(data.data);
    return data;
  } catch (error) {
    // Handle any errors that occur during the fetch operation
    console.error('Error fetching data:', error.message);
    throw error; // Rethrow the error to handle it elsewhere if needed
  }
}

const socket = new WebSocket('ws://localhost:8765');
const batteryFromDrone = document.querySelector(".battery-percentage")

// Handle messages received from the server
socket.onmessage = function (event) {
  const data = JSON.parse(event.data);
  // Do something with the received data, such as updating your 3D model orientation
  // console.log('Received orientation data:', data);
  const new_orientation_x = data.orientation_rpy_leveled[0];
  const new_orientation_y = data.orientation_rpy_leveled[1];
  const new_orientation_z = data.orientation_rpy_leveled[2];

  target_orientation_x = new_orientation_x;
  target_orientation_y = new_orientation_y;
  target_orientation_z = new_orientation_z;
  console.log('Data', data.orientation_rpy);
  // batteryFromDrone = data;
  if(data.battery_voltages ){
var batteryPercentage = (100-Number(batteryFromDrone.innerHTML)); // Example battery percentage
// Get the battery icon wrapper element
const batteryWrapper = document.querySelector('.battery-icon_wrapper');

// Determine the battery level based on the percentage
let batteryLevel;

voltage_1.innerHTML = Number(data.battery_voltages[0]).toFixed(1) + ' V cell 1'
voltage_2.innerHTML = Number(data.battery_voltages[1]).toFixed(1) + ' V cell 2'
voltage_3.innerHTML = Number(data.battery_voltages[2]).toFixed(1) + ' V cell 3'

  // batteryFromDrone.innerHTML = Number((data.battery_voltages[3]/ 12.6) * 100).toFixed(0) ;
  batteryFromDrone.innerHTML = Number((data.battery_lvl[3]).toFixed(3)*100).toFixed(1) ;
  batteryPercentage = (100-Number(batteryFromDrone.innerHTML)); // Example battery percentage
  if (batteryPercentage >= 80) {
    batteryLevel = 'lvl1';
  } else if (batteryPercentage >= 60) {
    batteryLevel = 'lvl2';
  } else if (batteryPercentage >= 40) {
    batteryLevel = 'lvl3';
  } else if (batteryPercentage >= 20) {
    batteryLevel = 'lvl4';
  } else {
    batteryLevel = 'lvl5';
  }
  console.log(batteryFromDrone.innerHTML)
// Add the appropriate class to the battery wrapper element
batteryWrapper.classList.add(batteryLevel);
  }
};

// Handle errors
socket.onerror = function (error) {
  console.error('WebSocket error:', error);
};

// Handle closure of the WebSocket connection
socket.onclose = function (event) {
  console.log('WebSocket connection closed:', event);
};




