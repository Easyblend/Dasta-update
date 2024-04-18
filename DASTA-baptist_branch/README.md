# DASTA

Based on the provided workspace structure and the code excerpt from main.py, it appears that this project is a real-time system for processing and visualizing sensor data from a device, possibly a drone or another type of remote-controlled vehicle.

Here's a brief overview of the project based on the files:

bluetoothTransmission.py: This file likely handles the Bluetooth communication between the device and the system. It sends commands to the device and receives sensor data.

calibration.py: This file likely contains code for calibrating the sensor data received from the device.

main.py: This is the main entry point of the system. It starts the Bluetooth transmission, processes the received data, and links the calibration and visualization.

myKalmanFilter.py: This file likely contains an implementation of a Kalman filter, a common algorithm used in navigation and control systems for estimating the state of a system from noisy measurements, used during the calibration process.

visu.py: This file likely contains code for visualizing the sensor data and/or the state of the device.



The real time data base directory contains JSON files (telemetry.json, userCommand.json), which are only for now examples of the data that the system will receive from the device


To do :
- [ ] implement the 2 data bases writing and reading