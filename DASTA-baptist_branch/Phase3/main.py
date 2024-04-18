import bluetoothTransmission
import time
import numpy as np
import calibration
# import progressbar
import gamecontroller
# import FakeDrone
import json
import os


dataBase_fileName = 'Phase3\dataBase.json'
default_content ={
    'bluetooth_address': '08:D1:F9:CE:C3:76',
    'ir leds': {
        'led1': {
            'position': [0.1351, -0.1380, -0.01791],
            'noise': 1
        },
        'led2': {
            'position': [0.1351, 0.1380, -0.01791],
            'noise': 1
        },
        'led3': {
            'position': [-0.1365, 0.1380, -0.01214],
            'noise': 1
        },
        'led4': {
            'position': [-0.1365, -0.1380, -0.01214],
            'noise': 1
        }
    },
    'calibration':
        {
        'last_calibration':
            {
                'date':None,
                'data':None
            },
        'initilization':
            {
                'environment' :
                    {
                    'gravity':9.812,
                    },
                'critical state':
                    {
                    'orientation_covariance':0.1,
                    },
                'imu':
                    {
                'gyro_noise':0.0017,
                'gyro_std_bias':0.03,
                'gyro_std_cross':0.01,
                'gyro_std_gain':0.01,
                'acc_noise':0.1,
                'acc_std_bias':0.7848,
                'acc_std_cross':0.01,
                'acc_std_gain':0.01
                    },
                'cameras':
                    {
                        'camera1' :
                        {
                            'id' : 1,
                            'brightness': 100,
                            'contrast': 16,
                            'fov' : 120,
                            'fov_std' : 1,
                            'distorion' : 0.16,
                            'distorion std' : 0.01,
                            'position' : [-2,-2,-2.7],
                            'position std' : [0.1,0.1,0.1],
                            'orientation rpy' : [0,45,45],
                            'orientation rpy std' : [10, 10, 10]
                        },
                        'camera2':
                        {
                            'id' : 2,
                            'brightness': 100,
                            'contrast': 17,
                            'fov' : 120,
                            'fov_std' : 1,
                            'distorion' : 0.16,
                            'distorion std' : 0.01,
                            'position' : [2,-2,-2.7],
                            'position std' : [0.1,0.1,0.1],
                            'orientation rpy' : [0,45,-135],
                            'orientation rpy std' : [10, 10, 10]
                        }
                    },

            },
        'tolerance':
            {
                'gyro_bias':0.0003,
                'acc_bias':0.005,
                'gyro_ortho':0.005,
                'acc_ortho':0.005,
                'camera position':0.01,
                'camera orientation': 0.5,
                'camera fov': 0.5,
                'camera distortion': 0.005,
                
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


running_calib = False
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
    ledsSettings = dataBase_content['ir leds']
    print(ledsSettings)
    calibration.ir_leds = []
    for ledname in ledsSettings.keys():
        calibration.ir_leds.append(calibration.led(ledsSettings[ledname]["position"],ledsSettings[ledname]["noise"]))
    
    calib_imu_settings = dataBase_content['calibration']['initilization']['imu']
    calib_env_settings = dataBase_content['calibration']['initilization']['environment']
    calib_critical_state_settings = dataBase_content['calibration']['initilization']['critical state']
    calib_cam_settings = dataBase_content['calibration']['initilization']['cameras']
    calibration.env.gravity = calib_env_settings['gravity']
    calibration.criticalState.setOriCov(calib_critical_state_settings['orientation_covariance'])
    calibration.imu.setGyrNoise(calib_imu_settings['gyro_noise'])
    calibration.imu.setGyrBiasCov(calib_imu_settings['gyro_std_bias'])
    calibration.imu.setGyrOrthoCov(calib_imu_settings['gyro_std_cross'],calib_imu_settings['gyro_std_gain'])
    calibration.imu.setAccNoise(calib_imu_settings['acc_noise'])
    calibration.imu.setAccBiasCov(calib_imu_settings['acc_std_bias'])
    calibration.imu.setAccOrthoCov(calib_imu_settings['acc_std_cross'],calib_imu_settings['acc_std_gain'])
    calibration.imu.gyr_bias_std_tol = dataBase_content['calibration']['tolerance']['gyro_bias']
    calibration.imu.acc_bias_std_tol = dataBase_content['calibration']['tolerance']['acc_bias']
    calibration.imu.gyr_ortho_std_tol = dataBase_content['calibration']['tolerance']['gyro_ortho']
    calibration.imu.acc_ortho_std_tol = dataBase_content['calibration']['tolerance']['acc_ortho']
    calibration.cameras = []
    for camera in calib_cam_settings.keys():
        new_cam = calibration.WideFoVCamera()
        new_cam.setIndex(calib_cam_settings[camera]['id'])
        new_cam.setK(calib_cam_settings[camera]['fov'])
        new_cam.setKcov(calib_cam_settings[camera]['fov_std'])
        new_cam.distortion_coefficient = calib_cam_settings[camera]['distorion']
        new_cam.setDistortionCoefficientCov(calib_cam_settings[camera]['distorion std'])
        new_cam.position = np.array(calib_cam_settings[camera]['position'])
        new_cam.setPosCov(np.array(calib_cam_settings[camera]['position std']))
        new_cam.setOrientation(np.array(calib_cam_settings[camera]['orientation rpy']))
        new_cam.setOriCov(np.array(calib_cam_settings[camera]['orientation rpy std']))
        new_cam.setBrightness(calib_cam_settings[camera]['brightness'])
        
        # new_cam.setBrightness(calib_cam_settings[camera]['brightness'])
        # new_cam.setContrast(calib_cam_settings[camera]['contrast'])
        calibration.cameras.append(new_cam)
        
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
                    print("Calibration progress: ", round(calibration.imu.gyr_bias_std_tol/calibration.imu.gyr_bias_cov_indicator(),3), " ", round(calibration.imu.acc_bias_std_tol/calibration.imu.acc_bias_cov_indicator(),3), " ", round(calibration.imu.gyr_ortho_std_tol/calibration.imu.gyr_ortho_cov_indicator(),3), " ", round(calibration.imu.acc_ortho_std_tol/calibration.imu.acc_ortho_cov_indicator(),3),[round(calibration.cameras[i].position_tolerance/calibration.cameras[i].position_cov_indicator(),3) for i in range(len(calibration.cameras))],[round(calibration.cameras[i].q_tolerance/calibration.cameras[i].orientation_cov_indicator(),3) for i in range(len(calibration.cameras))],[round(calibration.cameras[i].k_tolerance/calibration.cameras[i].k_cov_indicator(),3) for i in range(len(calibration.cameras))],[round(calibration.cameras[i].distortion_coefficient_tolerance/calibration.cameras[i].distortion_coefficient_cov_indicator(),3) for i in range(len(calibration.cameras))])
                if calibration.imu.calibrated():
                    running_calib = False
                    new_imu_calib = {}
                    
                    new_imu_calib["gyro_bias_co"] = calibration.imu.gyr_bias_co.tolist()
                    new_imu_calib["acc_bias_co"] = calibration.imu.acc_bias_co.tolist()
                    new_imu_calib["gyro_scale_co"] = calibration.imu.gyr_ortho_co.reshape(3,3).tolist()
                    new_imu_calib["acc_scale_co"] = calibration.imu.acc_ortho_co.reshape(3,3).tolist()
                    new_cam_calib = {}
                    for i in range(len(calibration.cameras)):
                        new_cam_calib["camera"+str(i+1)] = {}
                        new_cam_calib["camera"+str(i+1)]["position"] = calibration.cameras[i].position.tolist()
                        new_cam_calib["camera"+str(i+1)]["orientation"] = calibration.cameras[i].orientation.tolist()
                        new_cam_calib["camera"+str(i+1)]["k"] = calibration.cameras[i].k
                        new_cam_calib["camera"+str(i+1)]["distortion"] = calibration.cameras[i].distortion_coefficient
                        
                    modify_calibration({"imu":new_imu_calib,"cameras":new_cam_calib})
                    bluetoothTransmission.enable_stream_channel([bluetoothTransmission.receive_head[i]["name"] for i in range(len(bluetoothTransmission.receive_head))])
                    bluetoothTransmission.disable_stream_channel(["internal_event"])
                    bluetoothTransmission.data_to_send.append({"send_stream_delay_ms" : normal_sensor_stream_delay})
                    print("Calibration done")
        if ("internal_event" in data):
            print("Internal event received: ", find_key(bluetoothTransmission.embedded_event_dict, data["internal_event"]))
    #flush the received data
    bluetoothTransmission.received_data = []
    

def setup():
    dataBase_content = json.load(dataBase)
    bluetoothTransmission.fake_transmission = useFakeDrone
    bluetoothTransmission.init_transmission(dataBase_content['bluetooth_address'])

    if dataBase_content['calibration']['last_calibration']['date'] is None or time.time() - time.mktime(time.strptime(dataBase_content['calibration']['last_calibration']['date'], "%Y-%m-%d %H:%M:%S")) > 86400:
        print("Starting a new calibration because the last one was done more than 1 day ago or never done")
        begin_calibration(dataBase_content)
    else:
        print("Using the last calibration data")
        bluetoothTransmission.data_to_send.append(dataBase_content['calibration']['last_calibration']['data']['imu'])
        bluetoothTransmission.data_to_send.append(dataBase_content['calibration']['last_calibration']['data']['cameras'])
        bluetoothTransmission.data_to_send.append(dataBase_content['ir leds'])
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

if __name__ == "__main__":
    import cv2
    setup()
    while True:
        bluetoothTransmission.run_transmission()
        if len(bluetoothTransmission.received_data) > 0:
            on_received_data()
        new_cam = False
        for cam in calibration.cameras:
            if cam.read():
                new_cam = True
            screenName = 'cam'+str(cam.index)
            frame = cam.buid_desription_frame()
            if frame is not None:
                cv2.imshow(screenName,frame)
        if new_cam:
            calibration.update()
        
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
                
        gamecontroller.run()
        pack_command()
        time.sleep(0.001)
    cv2.destroyAllWindows()
    dataBase.close()
