import bluetoothTransmission
import calibration
import irCam
import time
from irCam import cv2
from irCam import np
import threading
'''
CALIBRATION INITIAL STATE
environment :
gravity = 9.812


critical state :
position = [0,0,0]
velocity = [0,0,0]
orientation = quaternion(1,0,0,0)

mpu6050 :
gyr_noise = 0.1*np.pi/180 = 0.0017
gyr_bias_tolerance = 0.03
gyr_cross_tolerance = 0.02
gyr_gain_tolerance = 0.03
acc_noise = 0.008*9.81 = 0.0785
acc_bias_tolerance = 0.08*9.81 = 0.7848
acc_gain_tolerance = 0.03
acc_cross_tolerance = 0.03

leds = [
    position = [0.03,0.02,0],
    position = [-0.03,-0.02,0]
]

camera :
position = [0.85,-0.85,-2.13]
orientation = EULER(rz = 135°, ry = -36°, rx = 0°)
k = 480
mounting position tolerance = 0.1m
mounting orientation tolerance = 5°
k_tolerance = 1



'''

def setup():
    calibration.env= calibration.environment()
    calibration.imu = calibration.MPU9250()
    calibration.leds = [calibration.led() for i in range(2)]
    calibration.cams = [calibration.camera() for i in range(1)]
    calibration.criticalState = calibration.CriticalState()
    
    calibration.env.gravity = 9.812
    q0, qvec = calibration.angle2quat(0,-180,0, input_unit='deg')
    calibration.criticalState.position = [0.7,-0.7,0]
    calibration.criticalState.velocity = [0,0,0]
    calibration.criticalState.orientation = calibration.Quaternion(q0,qvec[0],qvec[1],qvec[2])
    calibration.criticalState.setPosCov(0.05)
    calibration.criticalState.setVelCov(0.01)
    calibration.criticalState.setOriCov(5/180.0*3.14)
    
    calibration.imu.setGyrNoise(0.0017)
    calibration.imu.setGyrBiasCov(0.03)
    calibration.imu.setGyrOrthoCov(0.02,0.03)
    calibration.imu.setAccNoise(0.0785)
    calibration.imu.setAccBiasCov(0.7848)
    calibration.imu.setAccOrthoCov(0.03,0.03)
    
    calibration.leds[0].mounting_position = [0.03,-0.02,0.018]
    calibration.leds[1].mounting_position = [-0.03,0.02,0.018]
    
    calibration.cams[0].position = [2,-2,-2.60]
    q0,qvec = calibration.angle2quat(-134,0,41, input_unit='deg') 

    calibration.cams[0].orientation = calibration.Quaternion(q0,qvec[0],qvec[1],qvec[2])
    calibration.cams[0].k = 0
    calibration.cams[0].setPosCov(0.1)
    calibration.cams[0].setOriCov(5/180.0*3.14)
    calibration.cams[0].setKCov(1)
    calibration.cams[0].led_measurement_cov = np.eye(2)
    
'''
box dimensions to display
    x = 0.055
    y = 0.07
    z = 0.035    
'''

def draw_box(img):
    #project the box on the image using the calibration.prject function
    dx = 0.07
    dy = 0.055
    dz = 0.035
    vertices = np.array([[-dx/2,-dy/2,-dz/2],
                            [dx/2,-dy/2,-dz/2],
                            [dx/2,dy/2,-dz/2],
                            [-dx/2,dy/2,-dz/2],
                            [-dx/2,-dy/2,dz/2],
                            [dx/2,-dy/2,dz/2],
                            [dx/2,dy/2,dz/2],
                            [-dx/2,dy/2,dz/2]])
    #project the vertices
    projected = []
    for vertex in vertices:
        projected.append(calibration.project(vertex,calibration.criticalState.orientation.elements,calibration.criticalState.position,calibration.cams[0].position,calibration.cams[0].orientation.elements,calibration.cams[0].k))
        # projected.append(calibration.hcmln(calibration.my_ekf.x,0,vertex))
    # print(projected[0], img.shape)
    #draw the lines
    for i, j in [(0, 1), (1, 2), (2, 3), (3, 0),
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7)]:
            cv2.line(img,
                            (round(projected[i][0]+img.shape[1]/2),
                            round(projected[i][1]+img.shape[0]/2)),
                            (round(projected[j][0]+img.shape[1]/2),
                            round(projected[j][1]+img.shape[0]/2)), (255,255,255), 1)

            
       

def main():
    setup()
    cap = irCam.init()#init the camera (can take a few seconds)
    calibration.init()#init the calibration
    
    while not bluetoothTransmission.inited:
        time.sleep(0.1)
    
    time.sleep(1)
    bluetoothTransmission.data_to_send.append({"user_event": 7, "send_stream_delay": 20}) #ENABLE_SENSOR_STREAM
    time.sleep(3)
    bluetoothTransmission.data_to_send.append({"user_event": 3}) #START_STREAM

    camInited = False
    last_entity_points = None
    last_frame = None
    while(True):
        # if len(bluetoothTransmission.received_data) > 0:
        #     print(bluetoothTransmission.received_data)
        #     bluetoothTransmission.received_data = []
        for buffer in bluetoothTransmission.received_data:
            for data in buffer:
                if data is None:
                    continue
                if "gyro" in data and "acc" in data and "time" in data:
                    calibration.imu.time = data["time"]/1000
                    calibration.imu.new_gyr_sample = data["gyro"]
                    calibration.imu.new_acc_sample = data["acc"]
                    if camInited:
                        calibration.predict()
        bluetoothTransmission.received_data = []
        
        
        entity_points, originalFrame = irCam.main(cap)
        calibration.cams[0].fresh_led_measurements = entity_points
        if originalFrame is not None:
            last_frame = originalFrame.copy()
            if calibration.cams[0].k == 0:
                calibration.cams[0].k = 2*np.max(last_frame.shape)/(2*np.pi/3)
                calibration.cams[0].imageShape = last_frame.shape
            for segment in entity_points:
                for point in segment:
                    tmp = point[0]
                    point[0] = point[1] - last_frame.shape[1]/2
                    point[1] = tmp - last_frame.shape[0]/2
            
            if camInited:
                if entity_points is not None:
                    last_entity_points = entity_points.copy()
                    calibration.update()
        
        if last_entity_points is None:
            last_entity_points = []
        if last_frame is None:
            last_frame = np.zeros((480,640,3), dtype=np.uint8)
        else: 
            last_frame = cv2.normalize(last_frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)


        draw_box(last_frame)
        cv2.putText(last_frame,str(len(last_entity_points))+ "/"+str(irCam.ledNumber),(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
        #et on les ajoute sur l'image
        for segment in last_entity_points:
            for point in segment:
                cv2.circle(last_frame,(round(point[0]+last_frame.shape[1]/2),round(point[1]+last_frame.shape[0]/2)), 3, (0,0,255), -1)

        

        
        key = cv2.waitKey(1)
        if camInited:
            if key == ord('q'):
                setup()
                calibration.calib2X(calibration.Q, calibration.my_ekf.x, calibration.my_ekf.P, calibration.criticalState, calibration.imu, calibration.cams)
                camInited = False
        else:
            #on affiche la prediction des leds
            led1 = calibration.project(calibration.leds[0].mounting_position,calibration.criticalState.orientation.elements,calibration.criticalState.position,calibration.cams[0].position,calibration.cams[0].orientation.elements,calibration.cams[0].k)
            led2 = calibration.project(calibration.leds[1].mounting_position,calibration.criticalState.orientation.elements,calibration.criticalState.position,calibration.cams[0].position,calibration.cams[0].orientation.elements,calibration.cams[0].k)
            cv2.circle(last_frame,(round(led1[0]+last_frame.shape[1]/2),round(led1[1]+last_frame.shape[0]/2)), 3, (0,255,0), 1)
            cv2.circle(last_frame,(round(led2[0]+last_frame.shape[1]/2),round(led2[1]+last_frame.shape[0]/2)), 3, (0,255,0), 1)
                            
            #on affiche aussi la position et orientation de la camera
            txt = "camera position: "+str(calibration.cams[0].position)
            cv2.putText(last_frame,txt,(50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.3,(255,255,255),1,cv2.LINE_AA)
            txt = "camera orientation: "+str(calibration.cams[0].getRzyx())
            cv2.putText(last_frame,txt,(50,150), cv2.FONT_HERSHEY_SIMPLEX, 0.3,(255,255,255),1,cv2.LINE_AA)

            #modification en direct de la position de la camera
            if key == ord('z'):
                calibration.cams[0].position[2] += 0.01
            if key == ord('Z'):
                calibration.cams[0].position[2] -= 0.01
            if key == ord('x'):
                calibration.cams[0].position[0] += 0.01
            if key == ord('X'):
                calibration.cams[0].position[0] -= 0.01
            if key == ord('y'):
                calibration.cams[0].position[1] += 0.01
            if key == ord('Y'):
                calibration.cams[0].position[1] -= 0.01
                
            rzyx = calibration.cams[0].getRzyx()
            
            if key == ord('a'):
                rzyx[0] += 1
                calibration.cams[0].setOri(rzyx)
            if key == ord('A'):
                rzyx[0] -= 1
                calibration.cams[0].setOri(rzyx)
            if key == ord('s'):
                rzyx[1] += 1
                calibration.cams[0].setOri(rzyx)
            if key == ord('S'):
                rzyx[1] -= 1
                calibration.cams[0].setOri(rzyx)
            if key == ord('d'):
                rzyx[2] += 1
                calibration.cams[0].setOri(rzyx)
            if key == ord('D'):
                rzyx[2] -= 1
                calibration.cams[0].setOri(rzyx)
            if key == ord('q'):
                camInited = True
            if key == ord('p'):
                #exit the program
                cv2.destroyAllWindows()
                cap.release()
                exit()
                
            calibration.calib2X(calibration.Q, calibration.my_ekf.x, calibration.my_ekf.P, calibration.criticalState, calibration.imu, calibration.cams)
        
        # resize the image *1.8
        toShow = cv2.resize(last_frame,(int(last_frame.shape[1]*1.8),int(last_frame.shape[0]*1.8)))
        cv2.imshow('frame',toShow)
        # show the normalalized image
        
if __name__ == "__main__":
    bluetoothTransmission.main()#init the bluetooth
    threading.Thread(target=main).start()