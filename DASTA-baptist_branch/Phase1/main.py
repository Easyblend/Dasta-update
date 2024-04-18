import cv2
import bluetoothTransmission
import time
import numpy as np
import threading
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


def main():
    # open a window with sliders to control the angular velocity
    #framesize of 300x300
    cv2.namedWindow("CONTROL")
    cv2.resizeWindow("CONTROL", 300, 1000)

    angularVel = [0.0, 0.0, 0.0]
    rxmin = -1.0
    rxmax = 1.0
    rymin = -1.0
    rymax = 1.0
    rzmin = -1.0
    rzmax = 1.0
    throttle = [0.0]
    cv2.createTrackbar("throttle", "CONTROL", 0, 100, lambda x: throttle.__setitem__(0, 0.4 * x / 100))
    #slider centered in the middle of the window
    cv2.createTrackbar("X", "CONTROL", 0, 100, lambda x: angularVel.__setitem__(0, rxmin + (rxmax - rxmin) * x / 100))
    cv2.createTrackbar("Y", "CONTROL", 0, 100, lambda x: angularVel.__setitem__(1, rymin + (rymax - rymin) * x / 100))
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



    


    while True:
        
        while not bluetoothTransmission.inited:
            cv2.waitKey(1)
        
        time.sleep(1)
        bluetoothTransmission.data_to_send.append({"user_event": 9,"send_stream_delay": 75}) #ENABLE_SENSOR_STREAM
        time.sleep(2)
        bluetoothTransmission.data_to_send.append({"user_event": 9}) #START_GYRO CALIBRATION
        time.sleep(6)
        bluetoothTransmission.data_to_send.append({"user_event": 10}) #STOP_GYRO CALIBRATION
        bluetoothTransmission.data_to_send.append({"user_event": 3}) #START_STREAM

        while(True):
            if len(bluetoothTransmission.received_data) > 0:
                # print(bluetoothTransmission.received_data)
                for buffer in bluetoothTransmission.received_data:
                    for data in buffer:
                        if "gyro" in data and "angular_vel_command" in data:
                            pX.multiplot([int(data["gyro"][0]*100), int(100*data["angular_vel_command"][0])], "X")
                            pY.multiplot([int(100*data["gyro"][1]), int(100*data["angular_vel_command"][1])], "Y")
                            pZ.multiplot([int(100*data["gyro"][2]), int(100*data["angular_vel_command"][2])], "Z")
                        if "m1PWM" in data and "m2PWM" in data and "m3PWM" in data and "m4PWM" in data:
                            pMotor.multiplot([int(100*data["m1PWM"]), int(100*data["m2PWM"]), int(100*data["m3PWM"]), int(100*data["m4PWM"])], "Motors")
                        
                bluetoothTransmission.received_data = []

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            if key == ord('s'):
                data_to_send = {}
                data_to_send["thrust_command"] = throttle[0]
                data_to_send["angular_velocity_command"] = angularVel
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