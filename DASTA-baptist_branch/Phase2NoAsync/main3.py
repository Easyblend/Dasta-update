import bluetoothTransmission
import time
import numpy as np
import calibration
# import progressbar
import gamecontroller
# import FakeDrone
import json
import os
import threading

from fastapi import FastAPI
import uvicorn
import sqlite3
import asyncio
from fastapi.middleware.cors import CORSMiddleware

 
dataBase_fileName = 'Phase2NoAsync\dataBase.json'
default_content ={
    'bluetooth_address': '08:D1:F9:CE:C3:76',
    'calibration':
        {
        'last_calibration':
            {
                'date':None,
                'data':None
            },
        'initilization':
            {
                'gravity':9.812,
                'orientation_covariance':0.1,
                'gyro_noise':0.0017,
                'gyro_std_bias':0.03,
                'gyro_std_cross':0.01,
                'gyro_std_gain':0.01,
                'acc_noise':0.1,
                'acc_std_bias':0.7848,
                'acc_std_cross':0.01,
                'acc_std_gain':0.01
            },
        'tolerance':
            {
                'gyro_bias':0.0003,
                'acc_bias':0.005,
                'gyro_ortho':0.005,
                'acc_ortho':0.005
            }
        },
    'user_control':
        {
            "remote_commands" : [0,0,0,0],
            "remote_angula_velocity_sensitive" : 0.5,
            "remote_yaw_sensitive" : 1.0,
            "remote_attitude_sensitive" : 0.1,
            "user_event": bluetoothTransmission.user_event_dict["EnableStream"],
            "send_stream_delay_ms": 50
        },
    'real_time_telemetry': None,
    'telemetry_history': None
    }
 
#read the dataBase file or create a new one if it does not exist
dataBase = None
try :
    dataBase = open(dataBase_fileName, 'r+')
except FileNotFoundError:
    dataBase = open(dataBase_fileName, 'w+')
    dataBase.write(json.dumps(default_content, indent=4))
    dataBase.seek(0)
   
# if the file is empty write the default content
if os.stat(dataBase_fileName).st_size == 0:
    dataBase.write(json.dumps(default_content, indent=4))
    dataBase.seek(0)
   
#clean the real_time_telemetry and telemetry_history
dataBase_content = json.load(dataBase)
dataBase_content['real_time_telemetry'] = None
dataBase_content['telemetry_history'] = None
dataBase.seek(0)
dataBase.write(json.dumps(dataBase_content, indent=4))
dataBase.truncate()
dataBase.seek(0)
 
 
running_calib = True
useFakeDrone = False
calibration_sensor_stream_delay = 20
normal_sensor_stream_delay = 50
 
last_entry = {}
datas = []
 
#modify the section ['calibration']['last_calibration'] of the dataBase
def modify_calibration(new_calib):
    dataBase.seek(0)
    dataBase_content = json.load(dataBase)
    dataBase_content['calibration']['last_calibration']['date'] = time.strftime("%Y-%m-%d %H:%M:%S")
    dataBase_content['calibration']['last_calibration']['data'] = new_calib
    dataBase.seek(0)
    dataBase.write(json.dumps(dataBase_content, indent=4))
    dataBase.truncate()
    dataBase.seek(0)
    bluetoothTransmission.data_to_send.append(new_calib)
 
first_ekf_iteration = True
def begin_calibration(dataBase_content):
    calib_settings = dataBase_content['calibration']['initilization']
    calibration.env.gravity = calib_settings['gravity']
    calibration.criticalState.setOriCov(calib_settings['orientation_covariance'])
    calibration.imu.setGyrNoise(calib_settings['gyro_noise'])
    calibration.imu.setGyrBiasCov(calib_settings['gyro_std_bias'])
    calibration.imu.setGyrOrthoCov(calib_settings['gyro_std_cross'],calib_settings['gyro_std_gain'])
    calibration.imu.setAccNoise(calib_settings['acc_noise'])
    calibration.imu.setAccBiasCov(calib_settings['acc_std_bias'])
    calibration.imu.setAccOrthoCov(calib_settings['acc_std_cross'],calib_settings['acc_std_gain'])
    calibration.imu.gyr_bias_std_tol = dataBase_content['calibration']['tolerance']['gyro_bias']
    calibration.imu.acc_bias_std_tol = dataBase_content['calibration']['tolerance']['acc_bias']
    calibration.imu.gyr_ortho_std_tol = dataBase_content['calibration']['tolerance']['gyro_ortho']
    calibration.imu.acc_ortho_std_tol = dataBase_content['calibration']['tolerance']['acc_ortho']
    calibration.init()
    global running_calib, first_ekf_iteration
    running_calib = True
    first_ekf_iteration = True
    bluetoothTransmission.enable_stream_channel(["time_us","gyro_raw","acc_raw"])
    bluetoothTransmission.data_to_send.append({"send_stream_delay_ms" : calibration_sensor_stream_delay})
 
def update_dataBase_on_received_data(data):
    last_data = data[-1]
    dataBase.seek(0)
    dataBase_content = json.load(dataBase)
    dataBase_content['real_time_telemetry'] = last_data
    #update the telemetry history
    if dataBase_content['telemetry_history'] is None:
        dataBase_content['telemetry_history'] = []
    data[-1]['remote_commands'] =  [gamecontroller.x, gamecontroller.y, gamecontroller.z, gamecontroller.thrust+gamecontroller.ThrustTilte]
    dataBase_content['telemetry_history']+= data
    #update also the gamecontroller data
    dataBase_content['user_control']['remote_commands'] = [gamecontroller.x, gamecontroller.y, gamecontroller.z, gamecontroller.thrust+gamecontroller.ThrustTilte]
    dataBase.seek(0)
    dataBase.write(json.dumps(dataBase_content, indent=4))
    dataBase.truncate()
    dataBase.seek(0)
 
def find_key(dictionnaire, valeur):
    for cle, val in dictionnaire.items():
        if val == valeur:
            return cle
    return None
 
def link_db():
    global datas, last_entry
    datas.append(bluetoothTransmission.received_data)
    last_entry = bluetoothTransmission.received_data[-1]
 
def on_received_data():
    global running_calib, first_ekf_iteration
    link_db()
    # update_dataBase_on_received_data(bluetoothTransmission.received_data)
   
    for data in bluetoothTransmission.received_data:
        # if "orientation_rpy" in data:
        #     print("Orientation: ", np.round(data["orientation_rpy"],3))
        # if "battery_lvl" in data:
        #     print("Battery level: ", data["battery_lvl"][-1])
        if "time_us" in data and "gyro_raw" in data and "acc_raw" in data:
            calibration.imu.time = data["time_us"]/1000000.0
            calibration.imu.new_gyr_sample = np.array(data["gyro_raw"])
            calibration.imu.new_acc_sample = np.array(data["acc_raw"])
            if running_calib:
                if first_ekf_iteration:
                    calibration.initAttitudeUsingAcc()
                    first_ekf_iteration = False
                else:
                    calibration.predict()
                    calibration.update()
                    print("Calibration progress: ", round(calibration.imu.gyr_bias_std_tol/calibration.imu.gyr_bias_cov_indicator(),3), " ", round(calibration.imu.acc_bias_std_tol/calibration.imu.acc_bias_cov_indicator(),3), " ", round(calibration.imu.gyr_ortho_std_tol/calibration.imu.gyr_ortho_cov_indicator(),3), " ", round(calibration.imu.acc_ortho_std_tol/calibration.imu.acc_ortho_cov_indicator(),3))
                if calibration.imu.calibrated():
                    running_calib = False
                    new_calib = {}
                    new_calib["gyro_bias_co"] = calibration.imu.gyr_bias_co.tolist()
                    new_calib["acc_bias_co"] = calibration.imu.acc_bias_co.tolist()
                    new_calib["gyro_scale_co"] = calibration.imu.gyr_ortho_co.reshape(3,3).tolist()
                    new_calib["acc_scale_co"] = calibration.imu.acc_ortho_co.reshape(3,3).tolist()
                    modify_calibration(new_calib)
                    bluetoothTransmission.enable_stream_channel([bluetoothTransmission.receive_head[i]["name"] for i in range(len(bluetoothTransmission.receive_head))])
                    bluetoothTransmission.disable_stream_channel(["internal_event"])
                    bluetoothTransmission.data_to_send.append({"send_stream_delay_ms" : normal_sensor_stream_delay})
                    print("Calibration done")
        if ("internal_event" in data):
            if (data["internal_event"] != bluetoothTransmission.embedded_event_dict["None2"]):
                print("Internal event received: ", find_key(bluetoothTransmission.embedded_event_dict, data["internal_event"]))
    #flush the received data
    bluetoothTransmission.received_data = []
   
 
def setup():
    dataBase_content = json.load(dataBase)
    bluetoothTransmission.fake_transmission = useFakeDrone
    bluetoothTransmission.init_transmission(dataBase_content['bluetooth_address'])
 
    if dataBase_content['calibration']['last_calibration']['date'] is None or time.time() - time.mktime(time.strptime(dataBase_content['calibration']['last_calibration']['date'], "%Y-%m-%d %H:%M:%S")) > 86400 and not running_calib:
        print("Starting a new calibration because the last one was done more than 1 day ago or never done")
        begin_calibration(dataBase_content)
    else:
        print("Using the last calibration data")
        bluetoothTransmission.data_to_send.append(dataBase_content['calibration']['last_calibration']['data'])
        bluetoothTransmission.enable_stream_channel([bluetoothTransmission.receive_head[i]["name"] for i in range(len(bluetoothTransmission.receive_head))])
        bluetoothTransmission.disable_stream_channel(["internal_event"])
        bluetoothTransmission.data_to_send.append({"send_stream_delay_ms" : normal_sensor_stream_delay})
   
    bluetoothTransmission.data_to_send.append({"user_event": bluetoothTransmission.user_event_dict["EnableStream"]})
    bluetoothTransmission.data_to_send.append({"user_event" : bluetoothTransmission.user_event_dict["SwitchToAttiMode"]})
 
 
last_time_pack_command = time.time()
def pack_command():
    global last_time_pack_command
    t = time.time()
    if t-last_time_pack_command < 0.02 or not gamecontroller.is_new_data():
        return
    last_time_pack_command = t
    bluetoothTransmission.data_to_send.append({"remote_commands" : [gamecontroller.x, gamecontroller.y, gamecontroller.z, gamecontroller.thrust+gamecontroller.ThrustTilte]})
    if (gamecontroller.engage):
        bluetoothTransmission.data_to_send.append({"user_event" : bluetoothTransmission.user_event_dict["EngageMotors"]})
        gamecontroller.engage = False
    if (gamecontroller.disengage):
        bluetoothTransmission.data_to_send.append({"user_event" : bluetoothTransmission.user_event_dict["DisengageMotors"]})
        gamecontroller.disengage = False
 
import asyncio
import websockets
 
 
# print(last_entry)
async def handle_websocket(websocket, path):
    # This function will handle incoming WebSocket connections
    print(last_entry)
    try:
        while True:
            # Simulate sending orientation data periodically
            # You would replace this with your actual logic to fetch orientation data
            data = {"data": last_entry}
           
            # Send the orientation data to the client
            await websocket.send(json.dumps(data['data']))
            # Wait for a short period before sending the next update
            await asyncio.sleep(0.02)  # Adjust the delay as needed
    except websockets.exceptions.ConnectionClosedError:
        pass
 
 
# Function to start the WebSocket server in a separate thread
def run_websocket_server():
    asyncio.set_event_loop(asyncio.new_event_loop())
    start_server = websockets.serve(handle_websocket, "localhost", 8765)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
 
if __name__ == "__main__":
    websocket_thread = threading.Thread(target=run_websocket_server)
    websocket_thread.daemon = True  # Set the thread as a daemon so it exits when the main thread exits
    websocket_thread.start()
    setup()
    while True:
        bluetoothTransmission.run_transmission()
        if len(bluetoothTransmission.received_data) > 0:
            on_received_data()
        gamecontroller.run()
        pack_command()