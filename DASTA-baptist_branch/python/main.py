import bluetoothTransmission
import calibration
import threading
import time
from enum import Enum
import visu
import irCam
import cv2

'''
    wich is in cpp :
        enum UserEvent
            {
                None,
                StartStateEstimate,
                StopStateEstimate,
                StartStream,
                StopStream,
                EnableStateEstimateStream,
                DisableStateEstimateStream,
                EnableSensorStream,
                DisableSensorStream,
            }; 
'''
class UserEvent(Enum):
    NONE = 0
    START_STATE_ESTIMATE = 1
    STOP_STATE_ESTIMATE = 2
    START_STREAM = 3
    STOP_STREAM = 4
    ENABLE_STATE_ESTIMATE_STREAM = 5
    DISABLE_STATE_ESTIMATE_STREAM = 6
    ENABLE_SENSOR_STREAM = 7
    DISABLE_SENSOR_STREAM = 8
    


def linkBL_and_Calibration():

    calibration.init()
    #waite until the bluetooth is connected
    while not bluetoothTransmission.inited:
        time.sleep(0.1)
    
    time.sleep(1)
    bluetoothTransmission.data_to_send.append({"user_event": UserEvent.ENABLE_SENSOR_STREAM.value, "send_stream_delay": 20})
    bluetoothTransmission.data_to_send.append({"user_event": UserEvent.START_STREAM.value})
    while True:
        #check bluetoothTransmission.received_data 
        # print("len(bluetoothTransmission.received_data): ", len(bluetoothTransmission.received_data))
        for buffer in bluetoothTransmission.received_data:
            for data in buffer:
                if data is None:
                    continue
                if "gyro" in data and "acc" in data and "time" in data:
                    calibration.imu.time = data["time"]/1000
                    calibration.imu.new_gyr_sample = data["gyro"]
                    calibration.imu.new_acc_sample = data["acc"]
                    calibration.predict()
                    calibration.update(True)
                    # print(calibration.cams[0].last_led_measurements)
        bluetoothTransmission.received_data = []
        
        time.sleep(0.001)                

def linkBL_and_Cam():
    cap = irCam.init()
    while True:
        entity_points, originalFrame = irCam.main(cap)
        if entity_points is None:
            continue
        
        calibration.cams[0].fresh_led_measurements = entity_points
        # calibration.update()
        
        cv2.imshow('frame',originalFrame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    
        

def linkCalib_and_Visu():
    #create the visu
    screen = visu.init()
    clock = visu.pygame.time.Clock()
    while True:
        # visu.draw_cube(visu.Quaternion(calibration.ekf.Xn[6], calibration.ekf.Xn[7], calibration.ekf.Xn[8], calibration.ekf.Xn[9]), calibration.ekf.Xn[0:3].reshape(3))
        # visu.draw_Cov(calibration.ekf.Pn)
        # visu.draw_state(calibration.ekf.Xn)
        visu.draw_cube(screen,calibration.criticalState.position*0, calibration.criticalState.orientation, calibration.cams[0].last_led_measurements, calibration.cams[0].position, calibration.cams[0].orientation, calibration.cams[0].k)
        # On affiche le résultat
        visu.pygame.display.flip()
        # On attend 10 ms avant de recommencer
        clock.tick(100)
        if visu.pygame.event.get(visu.pygame.QUIT):
            visu.pygame.quit()
            exit()

        
    
import matplotlib.pyplot as plt



    
if __name__ == '__main__':
    bluetoothTransmission.main()
    threading.Thread(target=linkBL_and_Calibration).start()
    threading.Thread(target=linkCalib_and_Visu).start()
    # threading.Thread(target=linkBL_and_Cam).start()
    
    # while not bluetoothTransmission.inited:
    #     time.sleep(0.1)
    # time.sleep(60)
    # #show the histogram of the calibration.dt_save
    # plt.hist(calibration.dt_save, bins=100)
    # plt.show()