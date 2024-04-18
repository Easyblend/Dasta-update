# import threading
# from fastapi import FastAPI
# import uvicorn
# import sqlite3
# import asyncio
# from fastapi.middleware.cors import CORSMiddleware

import bluetoothTransmission
import time
import numpy as np
import calibration
# import progressbar
import gamecontroller
# import FakeDrone
import json
import os 


dataBase_fileName = 'Phase2NoAsync\dataBase.json'
default_content ={
    'bluetooth_address': 'FC:F5:C4:27:09:16',
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
dataBase = open(dataBase_fileName, 'r+')
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

datas = []
last_entry = {}

def update_dataBase_on_received_data(data):
    global last_entry  # Use the global keyword to modify the global variable
    datas.append(data)
    last_data = data[-1]
    last_entry = last_data  # Assign the last_data to last_entry directly
    # Assuming dataBase is a file object
    dataBase.seek(0)
    dataBase_content = json.load(dataBase)
    dataBase_content['real_time_telemetry'] = last_data
    # Update the telemetry history
    if dataBase_content['telemetry_history'] is None:
        dataBase_content['telemetry_history'] = []
    dataBase_content['telemetry_history'] += data
    # Update the gamecontroller data
    dataBase_content['user_control']['remote_commands'] = [gamecontroller.x, gamecontroller.y, gamecontroller.z, gamecontroller.thrust + gamecontroller.ThrustTilte]
    dataBase.seek(0)
    dataBase.write(json.dumps(dataBase_content, indent=4))
    dataBase.truncate()
    dataBase.seek(0)

def find_key(dictionnaire, valeur):
    for cle, val in dictionnaire.items():
        if val == valeur:
            return cle
    return None

def on_received_data():
    global running_calib, first_ekf_iteration
    
    update_dataBase_on_received_data(bluetoothTransmission.received_data)
    
    for data in bluetoothTransmission.received_data:
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
    # bluetoothTransmission.init_transmission("FC:F5:C4:27:09:16")

    if dataBase_content['calibration']['last_calibration']['date'] is None or time.time() - time.mktime(time.strptime(dataBase_content['calibration']['last_calibration']['date'], "%Y-%m-%d %H:%M:%S")) > 86400 and running_calib:
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

'''
---------------------------------------------------------------------------------------------------------
here is where we create a server to listen to the data sent by the drone and then make api calls 
to read and write from the database
---------------------------------------------------------------------------------------------------------
'''
# app = FastAPI()

# #Something for the frontend you shouldn't worry about
# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=["http://localhost:5173"],  # Replace with your frontend origin
#     allow_credentials=True,
#     allow_methods=["GET", "POST", "PUT", "DELETE"],
#     allow_headers=["*"],
# )



# #create a table in the database if it doesnt exist
# conn = sqlite3.connect('data.db')
# c = conn.cursor()
# c.execute('''CREATE TABLE IF NOT EXISTS sensor_data (
#                 time_us INTEGER,
#                 gyro_raw_x REAL,
#                 gyro_raw_y REAL,
#                 gyro_raw_z REAL,
#                 acc_raw_x REAL,
#                 acc_raw_y REAL,
#                 acc_raw_z REAL,
#                 gyro_x REAL,
#                 gyro_y REAL,
#                 gyro_z REAL,
#                 acc_x REAL,
#                 acc_y REAL,
#                 acc_z REAL,
#                 orientation_rpy_leveled_x REAL,
#                 orientation_rpy_leveled_y REAL,
#                 orientation_rpy_leveled_z REAL,
#                 orientation_rpy_x REAL,
#                 orientation_rpy_y REAL,
#                 orientation_rpy_z REAL,
#                 orientation_q_w REAL,
#                 orientation_q_x REAL,
#                 orientation_q_y REAL,
#                 orientation_q_z REAL,
#                 velocity_x REAL,
#                 velocity_y REAL,
#                 velocity_z REAL,
#                 position_x REAL,
#                 position_y REAL,
#                 position_z REAL,
#                 motor_speed_1 REAL,
#                 motor_speed_2 REAL,
#                 motor_speed_3 REAL,
#                 motor_speed_4 REAL,
#                 angular_vel_command_x REAL,
#                 angular_vel_command_y REAL,
#                 angular_vel_command_z REAL,
#                 thrust_command REAL,
#                 rpy_command_x REAL,
#                 rpy_command_y REAL,
#                 rpy_command_z REAL,
#                 kalman_delay_s_1 REAL,
#                 kalman_delay_s_2 REAL,
#                 close_loop_delay_s_1 REAL,
#                 close_loop_delay_s_2 REAL,
#                 stream_delay_s_1 REAL,
#                 stream_delay_s_2 REAL
#             )''')


# conn.commit()

# # write the values in datas to the database
# @app.get("/send_to_db")
# def write_to_db():
#     conn = sqlite3.connect('data.db')
#     c = conn.cursor()

#     values = (
#     last_entry.get('time_us', 0),
#     *last_entry.get('gyro_raw', [0, 0, 0]),
#     *last_entry.get('acc_raw', [0, 0, 0]),
#     *last_entry.get('gyro', [0, 0, 0]),
#     *last_entry.get('acc', [0, 0, 0]),
#     *last_entry.get('orientation_rpy_leveled', [0, 0, 0]),
#     *last_entry.get('orientation_rpy', [0, 0, 0]),
#     *last_entry.get('orientation_q', [0, 0, 0, 0]),
#     *last_entry.get('velocity', [0, 0, 0]),
#     *last_entry.get('position', [0, 0, 0]),
#     *last_entry.get('motor_speeds', [0, 0, 0, 0]),
#     *last_entry.get('angular_vel_command', [0, 0, 0]),
#     last_entry.get('thrust_command', 0),
#     *last_entry.get('rpy_command', [0, 0, 0]),
#     *last_entry.get('kalman_delay_s', [0, 0]),
#     *last_entry.get('close_loop_delay_s', [0, 0]),
#     *last_entry.get('stream_delay_s', [0, 0])
# )

#     # Construct SQL query and execute
#     c.execute('''INSERT INTO sensor_data (time_us, gyro_raw_x, gyro_raw_y, gyro_raw_z, acc_raw_x, acc_raw_y, acc_raw_z, 
#              gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, orientation_rpy_leveled_x, orientation_rpy_leveled_y, 
#              orientation_rpy_leveled_z, orientation_rpy_x, orientation_rpy_y, orientation_rpy_z, orientation_q_w, 
#              orientation_q_x, orientation_q_y, orientation_q_z, velocity_x, velocity_y, velocity_z, position_x, 
#              position_y, position_z, motor_speed_1, motor_speed_2, motor_speed_3, motor_speed_4, angular_vel_command_x, 
#              angular_vel_command_y, angular_vel_command_z, thrust_command, rpy_command_x, rpy_command_y, rpy_command_z, 
#              kalman_delay_s_1, kalman_delay_s_2, close_loop_delay_s_1, close_loop_delay_s_2, stream_delay_s_1, 
#              stream_delay_s_2) 
#              VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 
#              ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''', values)

# # Commit the changes and close the connection
#     conn.commit()
#     conn.close()

#     return {"data": last_entry}


# @app.get("/")
# def read_root():
#     return {"data":datas}

# @app.get("/recent_data")
# def read_recent_data():
#     return {"data": last_entry}

# #run the server
# def run_server():
#     uvicorn.run(app, host="0.0.0.0", port=8000)

# import asyncio
# import websockets

# async def handle_websocket(websocket, path):
#     # This function will handle incoming WebSocket connections
#     print(last_entry)
#     try:
#         while True:
#             # Simulate sending orientation data periodically
#             # You would replace this with your actual logic to fetch orientation data
#             data = {"data": last_entry}
            
#             # Send the orientation data to the client
#             await websocket.send(json.dumps(data['data']))
            
#             # Wait for a short period before sending the next update
#             await asyncio.sleep(0.05)  # Adjust the delay as needed
#     except websockets.exceptions.ConnectionClosedError:
#         pass

# # Function to start the WebSocket server in a separate thread
# def run_websocket_server():
#     asyncio.set_event_loop(asyncio.new_event_loop())
#     start_server = websockets.serve(handle_websocket, "localhost", 8765)
#     asyncio.get_event_loop().run_until_complete(start_server)
#     asyncio.get_event_loop().run_forever()


if __name__ == "__main__":
    # threading.Thread(target=run_server).start()
    # websocket_thread = threading.Thread(target=run_websocket_server)
    # websocket_thread.daemon = True  # Set the thread as a daemon so it exits when the main thread exits
    # websocket_thread.start()
    setup()
    while True:
        bluetoothTransmission.run_transmission()
        if len(bluetoothTransmission.received_data) > 0:
            on_received_data()
        gamecontroller.run()
        pack_command()