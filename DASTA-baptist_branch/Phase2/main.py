import cv2
import bluetoothTransmission
import time
import numpy as np
import threading
import calibration

class Plotter:
	def __init__(self, plot_width, plot_height, num_plot_values):
	    self.width = plot_width
	    self.height = plot_height
	    self.color_list = [(255, 0 ,0), (0, 250 ,0),(0, 0 ,250),
	    			(0, 255 ,250),(250, 0 ,250),(250, 250 ,0),
	    			(200, 100 ,200),(100, 200 ,200),(200, 200 ,100)]
	    self.color  = []
	    self.val = []
	    self.plot = np.ones((self.height, self.width, 3))*255

	    for i in range(num_plot_values):
	    	self.color.append(self.color_list[i])

	# Update new values in plot
	def multiplot(self, val, label = "plot"):
		self.val.append(val)
		while len(self.val) > self.width:
			self.val.pop(0)

		self.show_plot(label)

    # Show plot using opencv imshow
	def show_plot(self, label):
		self.plot = np.ones((self.height, self.width, 3))*255
		cv2.line(self.plot, (0, int(self.height/2) ), (self.width, int(self.height/2)), (0,255,0), 1)
		for i in range(len(self.val)-1):
			for j in range(len(self.val[0])):
				cv2.line(self.plot, (i, int(self.height/2) - self.val[i][j]), (i+1, int(self.height/2) - self.val[i+1][j]), self.color[j], 1)

		cv2.imshow(label, self.plot)
		# cv2.waitKey(1)# 10 is the delay in ms

def setup():
    calibration.env= calibration.environment()
    calibration.imu = calibration.MPU9250()
    calibration.criticalState = calibration.CriticalState()
    calibration.env.gravity = 9.812
    q0, qvec = calibration.angle2quat(0,0,0, input_unit='deg')
    calibration.criticalState.orientation = calibration.Quaternion(q0,qvec[0],qvec[1],qvec[2])
    calibration.criticalState.setOriCov(5/180.0*3.14)
    
    calibration.imu.setGyrNoise(0.0017)
    calibration.imu.setGyrBiasCov(0.03)
    calibration.imu.setGyrOrthoCov(0.02,0.03)
    calibration.imu.setAccNoise(0.0785)
    calibration.imu.setAccBiasCov(0.7848)
    calibration.imu.setAccOrthoCov(0.03,0.03)
    calibration.init()

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
        projected.append(calibration.project(vertex,calibration.criticalState.orientation.elements,np.array([0,0,0]),np.array([-0.3,0,0]),np.array([ 0.5, 0.5, 0.5, 0.5 ]),300))
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
    # open a window with sliders to control the angular velocity
    #framesize of 300x300
    cv2.namedWindow("CONTROL")
    cv2.resizeWindow("CONTROL", 300, 1000)

    angularVel = [0.0, 0.0, 0.0]
    rpy_command = [0.0, 0.0, 0.0]
    rxmin = -1.0
    rxmax = 1.0
    rymin = -1.0
    rymax = 1.0
    rzmin = -1.0
    rzmax = 1.0
    throttle = [0.0]
    cv2.createTrackbar("throttle", "CONTROL", 0, 100, lambda x: throttle.__setitem__(0, 0.4 * x / 100))
    #slider centered in the middle of the window
    cv2.createTrackbar("X", "CONTROL", 0, 100, lambda x: rpy_command.__setitem__(0, rxmin + (rxmax - rxmin) * x / 100))
    cv2.createTrackbar("Y", "CONTROL", 0, 100, lambda x: rpy_command.__setitem__(1, rymin + (rymax - rymin) * x / 100))
    cv2.createTrackbar("Z", "CONTROL", 0, 100, lambda x: angularVel.__setitem__(2, rzmin + (rzmax - rzmin) * x / 100))
    # but with default values to 50
    cv2.setTrackbarPos("X", "CONTROL", 50)
    cv2.setTrackbarPos("Y", "CONTROL", 50)
    cv2.setTrackbarPos("Z", "CONTROL", 50)


    #text fill in for pid parameters
    #params = [kp,ki,kd,timeConstDerFilter,maxIntegral,min,max]
    pidX = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pidY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pidZ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    minkp = 0.0
    maxkp = 1.0
    minki = 0.0
    maxki = 1.0
    minkd = 0.0
    maxkd = 1.0
    mintimeConstDerFilter = 0.0
    maxtimeConstDerFilter = 1.0
    maxmaxIntegral = 1.0
    minmaxIntegral = 0.0
    minmin = -1.0
    maxmin = 0.0
    minmax = 0.0
    maxmax = 1.0

 
    cv2.createTrackbar("kpX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(0, minkp + (maxkp - minkp) * x / 100))
    cv2.createTrackbar("kiX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(1, minki + (maxki - minki) * x / 100))
    cv2.createTrackbar("kdX", "CONTROL", 0, 1000, lambda x: pidX.__setitem__(2, minkd + (maxkd - minkd) * x / 1000))
    cv2.createTrackbar("timeConstDerFilterX", "CONTROL", 0, 1000, lambda x: pidX.__setitem__(3, mintimeConstDerFilter + (maxtimeConstDerFilter - mintimeConstDerFilter) * x / 1000))
    cv2.createTrackbar("maxIntegralX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(4, minmaxIntegral + (maxmaxIntegral - minmaxIntegral) * x / 100))
    cv2.createTrackbar("dynamicX", "CONTROL", 0, 1000, lambda x: pidX.__setitem__(5,  x / 1000))

    
    cv2.createTrackbar("kpY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(0, minkp + (maxkp - minkp) * x / 100))
    cv2.createTrackbar("kiY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(1, minki + (maxki - minki) * x / 100))
    cv2.createTrackbar("kdY", "CONTROL", 0, 1000, lambda x: pidY.__setitem__(2, minkd + (maxkd - minkd) * x / 1000))
    cv2.createTrackbar("timeConstDerFilterY", "CONTROL", 0, 1000, lambda x: pidY.__setitem__(3, mintimeConstDerFilter + (maxtimeConstDerFilter - mintimeConstDerFilter) * x / 1000))
    cv2.createTrackbar("maxIntegralY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(4, minmaxIntegral + (maxmaxIntegral - minmaxIntegral) * x / 100))
    cv2.createTrackbar("dynamicY", "CONTROL", 0, 1000, lambda x: pidY.__setitem__(5, x / 1000))

    cv2.createTrackbar("kpZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(0, minkp + (maxkp - minkp) * x / 100))
    cv2.createTrackbar("kiZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(1, minki + (maxki - minki) * x / 100))
    cv2.createTrackbar("kdZ", "CONTROL", 0, 1000, lambda x: pidZ.__setitem__(2, minkd + (maxkd - minkd) * x / 1000))
    cv2.createTrackbar("timeConstDerFilterZ", "CONTROL", 0, 1000, lambda x: pidZ.__setitem__(3, mintimeConstDerFilter + (maxtimeConstDerFilter - mintimeConstDerFilter) * x / 1000))
    cv2.createTrackbar("maxIntegralZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(4, minmaxIntegral + (maxmaxIntegral - minmaxIntegral) * x / 100))
    cv2.createTrackbar("dynamicZ", "CONTROL", 0, 1000, lambda x: pidZ.__setitem__(5, x / 1000))
    
    
    





    pX = Plotter(400, 200, 2)
    pY = Plotter(400, 200, 2)
    pZ = Plotter(400, 200, 2)
    pMotor = Plotter(400, 200, 4)
    Pypr = Plotter(400, 200, 3)
    P_gyrBias = Plotter(400, 200, 3)
    P_accBias = Plotter(400, 200, 3)




    setup()
    firstEfkIt = True
    running_calib = True
    while True:
        
        while not bluetoothTransmission.inited:
            cv2.waitKey(1)
        
        time.sleep(1)
        bluetoothTransmission.data_to_send.append({"send_stream_delay": 100}) #ENABLE_SENSOR_STREAM
        time.sleep(2)
        bluetoothTransmission.data_to_send.append({"user_event": 9}) #START_GYRO CALIBRATION
        time.sleep(6)
        bluetoothTransmission.data_to_send.append({"user_event": 10}) #STOP_GYRO CALIBRATION
        # bluetoothTransmission.data_to_send.append({'gyr_bias_co': [0.010367624548344168, 0.0008170391139779215, -0.0051163203624667915], 'acc_bias_co': [0.09940091506533233, 0.23789565762715395, -0.5238692602609472], 'gyr_ortho_co': [0.9971754389219237, -0.06825162317523605, 0.04086530590761851, 0.011545527110457462, 1.0176765814649802, 0.04699423297987611, -0.020739403416659658, -0.055172550100188955, 0.9935215380529903], 'acc_ortho_co': [0.9930989796495836, -0.056708431187433385, 0.033388238156223125, 0.052006384352081325, 0.993089337607381, 0.04155541869715461, -0.04274848616397965, -0.04281442247246134, 0.980427011153661]})
        time.sleep(1)
        bluetoothTransmission.data_to_send.append({"user_event": 3}) #START_STREAM

        running_calib = False
        while(True):
            if len(bluetoothTransmission.received_data) > 0:
                for buffer in bluetoothTransmission.received_data:
                    data = buffer[-1]
                    # print(qqdata)
                    if data is None:
                        continue
                    if "time" in data:
                        if data["time"] is None:
                            continue
                        calibration.imu.time = data["time"]/1000
                    if "gyro_raw" in data and "angular_vel_command" in data:
                        # pX.multiplot([int(data["gyro"][0]*100), int(100*data["angular_vel_command"][0])], "X")
                        # pY.multiplot([int(100*data["gyro"][1]), int(100*data["angular_vel_command"][1])], "Y")
                        # pZ.multiplot([int(100*data["gyro"][2]), int(100*data["angular_vel_command"][2])], "Z")
                        calibration.imu.new_gyr_sample = np.array(data["gyro_raw"])
                        if not firstEfkIt and running_calib:
                            calibration.predict()
                    if "acc_raw" in data:
                        calibration.imu.new_acc_sample = np.array(data["acc_raw"])
                        if firstEfkIt:
                            #compute the initial orientation using the accelerometer(compute pitch and roll and convert to quaternion)
                            pitch = np.arctan2(-data["acc_raw"][0], np.sqrt(data["acc_raw"][1]**2 + data["acc_raw"][2]**2))
                            roll = np.arctan2(-data["acc_raw"][1], -data["acc_raw"][2])
                            q0, qvec = calibration.angle2quat(0, pitch, roll, input_unit='rad')
                            calibration.criticalState.orientation = calibration.Quaternion(q0,qvec[0],qvec[1],qvec[2])
                            calibration.criticalState.setOriCov(5/180.0*3.14)
                            calibration.calib2X(calibration.Q, calibration.my_ekf.x, calibration.my_ekf.P, calibration.criticalState, calibration.imu)
                            bluetoothTransmission.data_to_send.append({"orientation": calibration.criticalState.orientation.elements.tolist(), 'user_event': 1}) #SEND_CALIBRATION and StartStateEstimate
                            bluetoothTransmission.data_to_send.append({"user_event": 11}) #START_ATTITUDE_CONTROL
                            firstEfkIt = False
                            print("ypr :")
                            print(0, pitch, roll)
                        else:
                            if running_calib:
                                calibration.update()
                            # q0,qvec = calibration.my_ekf.x[0], calibration.my_ekf.x[1:4]
                            # ypr = calibration.quat2angle(q0,qvec)
                            # Pypr.multiplot([int(ypr[0]*30), int(ypr[1]*30), int(ypr[2]*30)], "YPR")
                            # P_accBias.multiplot([int(calibration.my_ekf.P[7,7]*1000), int(calibration.my_ekf.P[8,8]*1000), int(calibration.my_ekf.P[9,9]*1000)], "AccBias")
                            # P_gyrBias.multiplot([int(calibration.my_ekf.P[4,4]*1000), int(calibration.my_ekf.P[5,5]*1000), int(calibration.my_ekf.P[6,6]*1000)], "GyrBias")
                    # if "m1PWM" in data and "m2PWM" in data and "m3PWM" in data and "m4PWM" in data:
                    #     pMotor.multiplot([int(100*data["m1PWM"]), int(100*data["m2PWM"]), int(100*data["m3PWM"]), int(100*data["m4PWM"])], "Motors")
                    img = np.zeros((300,300,3), np.uint8)
                    
                    # add indicator of calibration state using calibration.imu.gyr_bias_calibrated(), calibration.imu.acc_bias_calibrated(), calibration.imu.gyr_ortho_calibrated(), calibration.imu.acc_ortho_calibrated()
                    cv2.putText(img, "GyrBiasCalibrated: " + str(calibration.imu.gyr_bias_calibrated()), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(img, "AccBiasCalibrated: " + str(calibration.imu.acc_bias_calibrated()), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(img, "GyrOrthoCalibrated: " + str(calibration.imu.gyr_ortho_calibrated()), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(img, "AccOrthoCalibrated: " + str(calibration.imu.acc_ortho_calibrated()), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    if calibration.imu.calibrated() and running_calib:
                        bluetoothTransmission.data_to_send.append({"orientation": calibration.criticalState.orientation.elements.tolist(), "gyr_bias_co": calibration.imu.gyr_bias_co.tolist(), "acc_bias_co": calibration.imu.acc_bias_co.tolist(), "gyr_ortho_co": calibration.imu.gyr_ortho_co.reshape(9).tolist(), "acc_ortho_co": calibration.imu.acc_ortho_co.reshape(9).tolist(), "user_event": 11}) #SEND_CALIBRATION and START_ATTITUDE_CONTROL
                        running_calib = False
                    if ("orientation" in data):
                        calibration.criticalState.orientation = calibration.Quaternion(data["orientation"][0],data["orientation"][1],data["orientation"][2],data["orientation"][3])
                    draw_box(img) 
                    cv2.imshow("Box", img)
                        
                bluetoothTransmission.received_data = []
            # print(calibration.criticalState, calibration.imu)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            if key == ord('s'):
                data_to_send = {}
                data_to_send["thrust_command"] = throttle[0]
                data_to_send["angular_velocity_command"] = angularVel
                data_to_send["rpy_command"] = rpy_command
                data_to_send["pidX_kp"] = pidX[0]
                data_to_send["pidX_ki"] = pidX[1]
                data_to_send["pidX_kd"] = pidX[2]
                data_to_send["pidX_timeConstDerFilter"] = pidX[3]
                data_to_send["pidX_maxIntegral"] = pidX[4]
                data_to_send["pidX_min"] = -pidX[5]
                data_to_send["pidX_max"] = pidX[5]
                data_to_send["pidY_kp"] = pidY[0]
                data_to_send["pidY_ki"] = pidY[1]
                data_to_send["pidY_kd"] = pidY[2]
                data_to_send["pidY_timeConstDerFilter"] = pidY[3]
                data_to_send["pidY_maxIntegral"] = pidY[4]
                data_to_send["pidY_min"] = -pidY[5]
                data_to_send["pidY_max"] = pidY[5]
                data_to_send["pidZ_kp"] = pidZ[0]
                data_to_send["pidZ_ki"] = pidZ[1]
                data_to_send["pidZ_kd"] = pidZ[2]
                data_to_send["pidZ_timeConstDerFilter"] = pidZ[3]
                data_to_send["pidZ_maxIntegral"] = pidZ[4]
                data_to_send["pidZ_min"] = -pidZ[5]
                data_to_send["pidZ_max"] = pidZ[5]
                
                bluetoothTransmission.data_to_send.append(data_to_send)
                print(data_to_send)
        
        #close the window
        cv2.destroyAllWindows()
        break
    #disconnect bluetooth
    bluetoothTransmission.disconnect()
    quit()
                
if __name__ == "__main__":
    bluetoothTransmission.main()#init the bluetooth
    threading.Thread(target=main).start()
    
'''
sending : {'orientation': [0.027126550981525274, -0.555293746695634, 0.8309963793332694, 0.01892207040844341], 'gyr_bias_co': [0.010367624548344168, 0.0008170391139779215, -0.0051163203624667915], 'acc_bias_co': [0.09940091506533233, 0.23789565762715395, -0.5238692602609472], 'gyr_ortho_co': [0.9971754389219237, -0.06825162317523605, 0.04086530590761851, 0.011545527110457462, 1.0176765814649802, 0.04699423297987611, -0.020739403416659658, -0.055172550100188955, 0.9935215380529903], 'acc_ortho_co': [0.9930989796495836, -0.056708431187433385, 0.033388238156223125, 0.052006384352081325, 0.993089337607381, 0.04155541869715461, -0.04274848616397965, -0.04281442247246134, 0.980427011153661], 'user_event': 11}
'''