import cv2
import bluetoothTransmission
import time
import numpy as np
import threading
import calibration
import progressbar
import sys
# import pygame
import gamecontroller
import FakeDrone

useFakeDrone = False

quad = FakeDrone.FakeQuad()
               

received_orientation = [1,0,0,0]
received_position = [0,0,0]
  

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
    calibration.imu.setGyrOrthoCov(0.01,0.01)
    calibration.imu.setAccNoise(0.1)
    calibration.imu.setAccBiasCov(0.7848)
    calibration.imu.setAccOrthoCov(0.01,0.01)
    
    calibration.imu.gyr_bias_std_tol = 0.0003
    calibration.imu.acc_bias_std_tol = 0.005
    calibration.imu.gyr_ortho_std_tol = 0.003
    calibration.imu.acc_ortho_std_tol = 0.003
        
    calibration.init()


    
running_calib = False# not useFakeDrone

def draw():
    while not bluetoothTransmission.inited and not useFakeDrone:
        time.sleep(0.1)
    time.sleep(2)
    



    # print("gyr_bias, acc_bias, gyr_ortho, acc_ortho")
    # print("\n\n\n\n")
    gyr_bias_bar = progressbar.ProgressBar(maxval=100, line_offset= 1)
    acc_bias_bar = progressbar.ProgressBar(maxval=100, line_offset= 2)
    gyr_ortho_bar = progressbar.ProgressBar(maxval=100, line_offset= 3)
    acc_ortho_bar = progressbar.ProgressBar(maxval=100, line_offset= 4)
    # Create a file descriptor for regular printing as well
    # print_fd = progressbar.LineOffsetStreamWrapper(lines=0, stream=sys.stdout)
    # assert print_fd

    
    while running_calib:
    #     clock.tick(FPS)
    #     gamecontroller.run()
    #     draw_attitude()
    #     #update display
    #     pygame.display.flip()
        
        gyr_bias_bar.update(min(100,int(100*calibration.imu.gyr_bias_std_tol/calibration.imu.gyr_bias_cov_indicator())))
        acc_bias_bar.update(min(100,int(100*calibration.imu.acc_bias_std_tol/calibration.imu.acc_bias_cov_indicator())))
        gyr_ortho_bar.update(min(100,int(100*calibration.imu.gyr_ortho_std_tol/calibration.imu.gyr_ortho_cov_indicator())))
        acc_ortho_bar.update(min(100,int(100*calibration.imu.acc_ortho_std_tol/calibration.imu.acc_ortho_cov_indicator())))
        # for event in pygame.event.get():
        #     if event.type == pygame.QUIT:
        #         pygame.quit()
        #         bluetoothTransmission.data_to_send.append({"user_event": 13}) #EMERGENCY_STOP
 
        
    
    gyr_bias_bar.finish()
    acc_bias_bar.finish()
    gyr_ortho_bar.finish()
    acc_ortho_bar.finish()
    print("gyr_bias, acc_bias, gyr_ortho, acc_ortho")
    print(calibration.imu.gyr_bias_co.tolist(), calibration.imu.acc_bias_co.tolist(), calibration.imu.gyr_ortho_co.reshape(9).tolist(), calibration.imu.acc_ortho_co.reshape(9).tolist())


def main():

    global received_orientation, running_calib, received_position
    if not running_calib:
    #    print("gyr_bias, acc_bias, gyr_ortho, acc_ortho")

        bluetoothTransmission.data_to_send.append({
            "gyr_bias_co":[-0.00764382870598456, -0.002906589515953307, -0.008359488215692632],
            "acc_bias_co" :[-0.5381701629683476, -0.16040564811189145, -0.7443503860074567],
            "gyr_ortho_co": [0.9771628295377841, -0.05299494120744518, -0.14232190400465064,
                            0.006196144105722445, 1.0019661037095184, -0.013380961482694914,
                            0.13818732367729306, -0.008985933421890196, 0.9884554847530128],
            "acc_ortho_co": [0.9575774950927878, -0.009070080431738493, -0.13996072437652746,
                            -0.004974358038731178, 1.0127180430455083, -0.03654834981032832,
                            0.13328585931628698, 0.02380419054852441, 0.964911122794868],
            "user_event": 1,
            "send_stream_delay": 50
        })
        bluetoothTransmission.data_to_send.append({"user_event": 11})
    time.sleep(3)

    last_time = time.time()
    setup()
    firstEfkIt = True
    while not bluetoothTransmission.inited and not useFakeDrone:
        cv2.waitKey(1)

    if not running_calib:
        bluetoothTransmission.data_to_send.append({"send_stream_delay": 10, "user_event": 3})
    
    while(True):
        new_kalmanIt = False
        to_send = {}
        if not useFakeDrone:
            for buffer in bluetoothTransmission.received_data:
                for data in buffer:
                    # print(data)
                    if data is None:
                        continue
                    if "orientation" in data:
                        # received_orientation = data["orientation"]
                        gamecontroller.display_attitude = data["orientation"]
                    if "time" in data and "gyro_raw" in data and "acc_raw" in data:
                        calibration.imu.time = data["time"]/1000
                        calibration.imu.new_gyr_sample = np.array(data["gyro_raw"])
                        calibration.imu.new_acc_sample = np.array(data["acc_raw"])
                        if firstEfkIt:
                            calibration.initAttitudeUsingAcc()
                            firstEfkIt = False
                        if running_calib:
                            calibration.predict()
                            calibration.update()
                            new_kalmanIt = True
                            gamecontroller.display_attitude = calibration.criticalState.orientation.elements.tolist()

                    if calibration.imu.calibrated() and running_calib:
                        to_send["gyr_bias_co"] = calibration.imu.gyr_bias_co.tolist()
                        to_send["acc_bias_co"] = calibration.imu.acc_bias_co.tolist()
                        to_send["gyr_ortho_co"] = calibration.imu.gyr_ortho_co.reshape(9).tolist()
                        to_send["acc_ortho_co"] = calibration.imu.acc_ortho_co.reshape(9).tolist()
                        to_send["user_event"] = 11 #StartAttitudeControl
                        to_send["send_stream_delay"] = 50
                        to_send["orientation"] = calibration.criticalState.orientation.elements.tolist()
                        running_calib = False

                
                # gamecontroller.display_position = calibration.criticalState.position.elements.tolist()
            #     to_send["orientation"] = calibration.criticalState.orientation.elements.tolist()
                # print("orientation : " + str(to_send["orientation"]))
        else:
            dt = time.time()-last_time
            last_time = time.time()
            quad.run(gamecontroller.x*0.5,gamecontroller.y*0.5,gamecontroller.z,gamecontroller.thrust+gamecontroller.ThrustTilte,dt)
            # received_orientation = quad.state.orientation.elements
            # received_position = quad.state.position
            gamecontroller.display_attitude = quad.state.orientation.elements.tolist()
            gamecontroller.display_position = quad.state.position.tolist()
            # received_orientation = quad.state.orientation.elements.tolist()
            received={}
            received["time"] = int(time.time()*1000)
            received["gyro_raw"] = quad.imu.gyr.tolist()
            received["acc_raw"] = quad.imu.acc.tolist()
            received["orientation"] = gamecontroller.display_attitude
            received["position"] = gamecontroller.display_position
            received["w1"] = quad.engines[0].u
            received["w2"] = quad.engines[1].u
            received["w3"] = quad.engines[2].u
            received["w4"] = quad.engines[3].u
            received["angular_velocity_command"] = quad.angular_vel_command.tolist()
            bluetoothTransmission.received_data.append(received)
        
        if not running_calib:
        #     # to_send["rpy"] = [gamecontroller.x*0.5,gamecontroller.y*0.5,0]
            to_send["angular_velocity_command"] = [gamecontroller.x,gamecontroller.y,gamecontroller.z]
            to_send["thrust_command"] = gamecontroller.thrust+gamecontroller.ThrustTilte
       
        if len(to_send.keys()) > 0:
            bluetoothTransmission.data_to_send = [to_send]
        #     print("sending : " + str(to_send))
        
        # print(bluetoothTransmission.received_data)
        # bluetoothTransmission.data_to_send = []
        bluetoothTransmission.received_data = [] 


        time.sleep(0.01)
    
if __name__ == "__main__":
    if not useFakeDrone:
        bluetoothTransmission.main()#init the bluetooth
    threading.Thread(target=main).start()
    threading.Thread(target=draw).start()
    gamecontroller.main()
    # threading.Thread(target=gamecontroller.main).start()
    # threading.Thread(target=gamecontroller.main).start()

'''
sending : {
'gyr_bias_co': [0.002801945970295527, 0.029011749265692822, 0.006528320068460152], 
'acc_bias_co': [0.051844492672723085, 0.25888215285081284, -0.16931503135376746], 
'gyr_ortho_co': [1.0001186968305638, 0.0035822877831092215, 0.05601770609824901,
                -0.01090682160310374, 0.9960287895051348, 0.020462629344681432, 
                -0.05782902041304632, 0.012969463821746484, 1.008285422856452], 
'acc_ortho_co': [0.9713035146466907, 0.011343414721059332, 0.0550657136657784, 
                0.007858472466955618, 0.9876143664476156, 0.0006895721787651755, 
                -0.06374875003554434, -0.014924327347036194, 0.9891866024945776], 
'user_event': 11, 'send_stream_delay': 50}

sending : {
'gyr_bias_co': [0.001480111875270763, 0.027379781436429575, 0.003728147959764632], 
'acc_bias_co': [-0.09066368609265504, 0.05678553569631716, -0.16818117885522724], 
'gyr_ortho_co': [0.9857398902753511, 0.1766693293253466, -0.012736265612045434, 
                -0.16526740325914183, 0.9807870201258838, -0.15380944713849254, 
                -0.029374779774986957, 0.17748753724523122, 0.9713055867862417], 
'acc_ortho_co': [0.9688797199187668, 0.17432720253430323, 0.02204680192852682, 
                -0.1620987331318013, 0.9792023908415275, -0.1450348774493668, 
                -0.01658107019549306, 0.1461910747069092, 0.9968784020748184], 
'user_event': 11, 'send_stream_delay': 50}
'''