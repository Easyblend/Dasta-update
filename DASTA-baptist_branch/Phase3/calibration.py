# to use an extended Kalman filter, we can use
import ekf
import numpy as np
from navpy import angle2quat
from navpy import quat2angle
from navpy import quat2dcm
# from navpy import dcm2quat
from pyquaternion import Quaternion
import time as t

'''
____________STATE ESTIMATE VECTOR______________

        State                   unit        index       length
orientation(quaternion)                     0:4         4
position(xyz)                   m           4:7         3
velocity(xyz)                   m/s         7:10         3
gyr_bias_correction(xyz)        rad/s       10:13        3
acc_bias_correction(xyz)        m/s²        13:16       3
gyr_ortho_correction(3*3)                   16:25       9
acc_ortho_correction(3*3)                   25:34       9

..............camera parameters..............
cam0_position(xyz)              m           34:37       3
cam0_orientation(quaternion)                37:41       4
cam0_k                          px          41          1
cam0_alpha                                  42          1
                         .
                         .
                         .
camN_position(xyz)              m           34+9N:37+9N 3
camN_orientation(quaternion)                37+9N:41+9N 4
camN_k                          px          41+9N       1
camN_alpha                                  42+9N       1



___________COMMANDS VECTOR_____________________

        State                   unit        index       length
gyroscopes(xyz)                 rad/s       0:3         3       
accelerometers(xyz)             m/s²        3:6         3


___________SENSOR VECTORS_____________________

..............CAMERA_i..............
        State                   unit       index        length
leds_position(uv)               px          0:2         2


'''           
class CriticalState:
        def __init__(self,orientation = Quaternion(1,0,0,0), position = np.zeros(3), velocity = np.zeros(3)):
                self.orientation = orientation          
                self.setOriCov(0)
                self.position = position
                self.setPosCov(0)
                self.velocity = velocity
                self.setVelCov(0)
                
                
                        
        def setOriCov(self, ori_accur):
                self.orientation_cov = np.eye(4)*ori_accur**2
        def setPosCov(self, pos_accur):
                self.position_cov = np.eye(3)*pos_accur**2
        def setVelCov(self, vel_accur):
                self.velocity_cov = np.eye(3)*vel_accur**2
                
        def orientation_calibration_indicator(self):
                return np.sqrt(np.max(np.diag(self.orientation_cov)))
        def orientation_calibrated(self, tolerance=0.01):
                return self.orientation_calibration_indicator()<tolerance
        def position_calibration_indicator(self):
                return np.sqrt(np.max(np.diag(self.position_cov)))
        def position_calibrated(self, tolerance=0.01):
                return self.position_calibration_indicator()<tolerance
        def velocity_calibration_indicator(self):
                return np.sqrt(np.max(np.diag(self.velocity_cov)))
        def velocity_calibrated(self, tolerance=0.01):
                return self.velocity_calibration_indicator()<tolerance
        
        def __str__(self) -> str:
                return "criticalState\norientation = "+str(self.orientation)+"\norientation_cov = "+str(self.orientation_cov) + "\nposition = "+str(self.position)+"\nposition_cov = "+str(self.position_cov) + "\nvelocity = "+str(self.velocity)+"\nvelocity_cov = "+str(self.velocity_cov)
                
                
class MPU9250:
        def __init__(self,gyr_noise=0,gyr_bias_tolerance=0,gyr_cross_tolerance=0,acc_noise=0,acc_bias_tolerance=0,acc_gain_tolerance=0,acc_cross_tolerance=0):               
                self.new_acc_sample = None
                self.new_gyr_sample = None
                self.time = None
                self.prev_time = None
                                
                self.acc_bias_co = np.zeros(3)
                self.gyr_bias_co = np.zeros(3)
                
                self.gyr_ortho_co = np.eye(3)
                self.acc_ortho_co = np.eye(3)

                
                self.setAccBiasCov(acc_bias_tolerance)
                self.setGyrBiasCov(gyr_bias_tolerance)
                
                self.setGyrNoise(gyr_noise)
                self.setAccNoise(acc_noise)
                
                self.setGyrOrthoCov(gyr_cross_tolerance,gyr_cross_tolerance)
                self.setAccOrthoCov(acc_cross_tolerance,acc_gain_tolerance)
                
                self.gyr_bias_co = np.zeros(3)
                self.acc_bias_co = np.zeros(3)
                
                self.gyr_ortho_co = np.eye(3)
                self.acc_ortho_co = np.eye(3)
                
                self.gyr_bias_std_tol = 0.01
                self.acc_bias_std_tol = 0.02
                self.gyr_ortho_std_tol = 0.005
                self.acc_ortho_std_tol = 0.005
                
                

        def setGyrNoise(self,gyr_noise):
                self.gyr_noise = np.eye(3)*gyr_noise**2
        
        def setAccNoise(self,acc_noise):
                self.acc_noise = np.eye(3)*acc_noise**2
                
        def setGyrOrthoCov(self,gyr_cross_tolerance,gyr_gain_tolerance):               
                self.gyr_ortho_cov = np.eye(9)*gyr_cross_tolerance**2 + np.diag((np.eye(3)*gyr_gain_tolerance**2).reshape(9))
        
        def setAccOrthoCov(self,acc_cross_tolerance,acc_gain_tolerance):
                self.acc_ortho_cov = np.eye(9)*acc_cross_tolerance**2 + np.diag((np.eye(3)*acc_gain_tolerance**2).reshape(9))
                
        def setGyrBiasCov(self,gyr_bias_tolerance):
                self.gyr_bias_cov = np.eye(3)*gyr_bias_tolerance**2
                
        def setAccBiasCov(self,acc_bias_tolerance):
                self.acc_bias_cov = np.eye(3)*acc_bias_tolerance**2
                
        def gyr_bias_cov_indicator(self):
                return np.sqrt(np.max(np.diag(self.gyr_bias_cov)))
        
        def acc_bias_cov_indicator(self):
                return np.sqrt(np.max(np.diag(self.acc_bias_cov)))
        
        def gyr_ortho_cov_indicator(self):
                return np.sqrt(np.max(np.diag(self.gyr_ortho_cov)))
        
        def acc_ortho_cov_indicator(self):
                return np.sqrt(np.max(np.diag(self.acc_ortho_cov)))
         
                
        def gyr_bias_calibrated(self):
                return self.gyr_bias_cov_indicator()<self.gyr_bias_std_tol
        
        def acc_bias_calibrated(self):
                return self.acc_bias_cov_indicator()<self.acc_bias_std_tol
        
        def gyr_ortho_calibrated(self):
                return self.gyr_ortho_cov_indicator()<self.gyr_ortho_std_tol
        
        def acc_ortho_calibrated(self):
                return self.acc_ortho_cov_indicator()<self.acc_ortho_std_tol
        
        def calibrated(self):
                return self.gyr_bias_calibrated() and self.acc_bias_calibrated() and self.gyr_ortho_calibrated() and self.acc_ortho_calibrated()
        
        def __str__(self) -> str:
                return "MPU9250\ngyr_noise = "+str(self.gyr_noise)+"\nacc_noise = "+str(self.acc_noise)+"\nacc_bias_co = "+str(self.acc_bias_co)+"\ngyr_bias_co = "+str(self.gyr_bias_co)+"\nacc_ortho_co = "+str(self.acc_ortho_co)+"\ngyr_ortho_co = "+str(self.gyr_ortho_co)+"\nacc_bias_cov = "+str(self.acc_bias_cov)+"\ngyr_bias_cov = "+str(self.gyr_bias_cov)+"\nacc_ortho_cov = "+str(self.acc_ortho_cov)+"\ngyr_ortho_cov = "+str(self.gyr_ortho_cov)
                           
                

class environment:
        def __init__(self, gravity=9.81):
                self.gravity = gravity
                
        def __str__(self) -> str:
                return "environment\ngravity = "+str(self.gravity)
    
class led:
        def __init__(self,mounting_position,noise=None):
                self.mounting_position = mounting_position
                self.setNoise(noise)
                
        def setNoise(self,std):
                if std is None:
                        self.noise = None
                        self.noise_cov = None
                else:
                        self.noise = std
                        self.noise_cov = np.eye(2)*std**2
                
        def __str__(self) -> str:
                return "led\nmounting_position = "+str(self.mounting_position)
                
import irCam
def draw_ellipse(cv2_frame, covariance_matrix, mean, color=(0, 255, 0), thickness=2):
        eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
        eigenvalues, eigenvectors = eigenvalues[0], eigenvectors[0]
        
        # Find major and minor axes lengths
        major_axis_length = np.sqrt(eigenvalues[0])
        minor_axis_length = np.sqrt(eigenvalues[1])
        
        # Compute rotation angle
        angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]) * 180.0 / np.pi
        
        # Convert angle to OpenCV convention
        if angle < 0:
                angle += 180
        
        # Draw ellipse
        cv2.ellipse(cv2_frame, 
                        (int(mean[0]), int(mean[1])), 
                        (int(major_axis_length), int(minor_axis_length)), 
                        angle, 
                        0, 
                        360, 
                        color, 
                        thickness)

import cv2
class WideFoVCamera:
        def __init__(self,index=None,position=None,orientation_rpy=None,fov_deg=None, distortion_coefficient=0.16):
                
                self.setIndex(index)      
                self.fresh_led_measurements = None
                self.last_led_measurements = None
                self.predicted_led_measurements = []
                self.predicted_led_measurements_cov = []
                
                self.position = position
                self.distortion_coefficient = distortion_coefficient
                self.distortion_coefficient_cov = 0.01
                self.setOrientation(orientation_rpy)
                self.setK(fov_deg)
                self.position_cov = np.eye(3)
                self.orientation_q_cov = np.eye(4)
                self.k_cov = 1
                self.led_measurement_cov = 1*np.eye(2)
                self.position_tolerance = 0.01
                self.q_tolerance = 0.01
                self.k_tolerance = 0.01
                self.distortion_coefficient_tolerance = 0.01
                
        def set_q_tolerance(self,deg):
                self.q_tolerance = deg/180*np.pi
                
        def set_k_tolerance(self,fov_deg_tol):
                self.k_tolerance = max(self.frame_shape[0],self.frame_shape[1])/fov_deg_tol*180/np.pi
        
        def position_cov_indicator(self):
                return np.sqrt(np.max(np.diag(self.position_cov)))

        def orientation_cov_indicator(self):
                return np.sqrt(np.max(np.diag(self.orientation_q_cov)))
        
        def k_cov_indicator(self):
                return np.sqrt(self.k_cov)
        
        def distortion_coefficient_cov_indicator(self):
                return np.sqrt(self.distortion_coefficient_cov)
        
        def setIndex(self,index):
                self.index = index
                if index is None:
                        self.cap = None
                        self.frame_shape = None
                else:
                        self.cap = irCam.init(index)
                        self.frame_shape = (self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        def setBrightness(self,brightness):
                self.cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)

        def setContrast(self,contrast):
                self.cap.set(cv2.CAP_PROP_CONTRAST, contrast)           

        def read(self):
                self.fresh_led_measurements, self.frame = irCam.main(self.cap)
                if self.fresh_led_measurements is not None:
                        return True
                return False
        
        def buid_desription_frame(self):
                if self.frame is None:
                        return None
                des_frame = self.frame.copy()
                #add the information of the camera (index, position, orientation, fov_deg, distortion_coefficient)
                cv2.putText(des_frame, "index = "+str(self.index), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                cv2.putText(des_frame, "position = "+str(self.position), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                cv2.putText(des_frame, "orientation = "+str(self.orientation_rpy), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                cv2.putText(des_frame, "fov_deg = "+str(self.fov_deg), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                cv2.putText(des_frame, "distortion_coefficient = "+str(self.distortion_coefficient), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                
                for segment in self.fresh_led_measurements:
                        for point in segment:
                                cv2.circle(des_frame,(round(point[0]+self.frame_shape[0]/2),round(point[1]+self.frame_shape[1]/2)), 3, (0,255,0), -1)
                
                #display the predicted led measurements
                for point in self.predicted_led_measurements:
                        # cv2.circle(des_frame,(round(point[1]+self.frame_shape[1]/2),round(point[0]+self.frame_shape[0]/2)), 5, (0,255,0), -1)
                        draw_ellipse(des_frame, self.predicted_led_measurements_cov, point+np.array([self.frame_shape[0]/2,self.frame_shape[1]/2]), color=(255, 0, 0), thickness=1)
                
                return des_frame
        
                     
        def setOrientation(self,orientation_rpy):
                if orientation_rpy is None:
                        self.orientation_q = None
                        self.orientation_rpy = None
                else:
                        self.orientation_rpy = orientation_rpy
                        q0,qvec = angle2quat(orientation_rpy[2],orientation_rpy[1],orientation_rpy[0],input_unit='deg',rotation_sequence='ZYX')
                        self.orientation_q = Quaternion(q0,qvec[0],qvec[1],qvec[2])
        
        def setPosCov(self,std):
                self.position_cov = np.diag(std)**2
        
        def setOriCov(self,std_deg):
                self.orientation_q_cov = np.eye(4)*(np.max(std_deg)/180*np.pi)**2
        
        def setDistortionCoefficientCov(self,std):
                if std is None:
                        self.distortion_coefficient_cov = None
                else :
                        self.distortion_coefficient_cov = std**2
                
        def setK(self,fov_deg):
                if fov_deg is None:
                        self.k = None
                        self.fov_deg = None
                        self.foV_rad = None
                        self.max_pix_r = None
                else:
                        self.k = max(self.frame_shape[0],self.frame_shape[1])/fov_deg*180/np.pi
                        self.fov_deg = fov_deg
                        self.foV_rad = fov_deg*np.pi/180
                        self.max_pix_r = max(self.frame_shape[0],self.frame_shape[1])/2

        
        def setKcov(self, fov_deg_std):
                self.k_cov = max(self.frame_shape[0],self.frame_shape[1]) * 180 / np.pi * fov_deg_std
        
        def setMesureCov(self,std):
                self.led_measurement_cov = np.eye(2)*std**2
        
        def position_calibration_indicator(self):
                return np.sqrt(np.max(np.diag(self.position_cov)))
        
        def orientation_calibration_indicator(self):
                return np.sqrt(np.max(np.diag(self.orientation_rpy_cov)))
        
        def k_calibration_indicator(self):
                return np.sqrt(self.k_cov)
        
        def position_calibrated(self,tolerance):
                return self.position_calibration_indicator()<tolerance
        
        def orientation_calibrated(self,tolerance):
                return self.orientation_calibration_indicator()<tolerance
        
        def k_calibrated(self,tolerance):
                return self.k_calibration_indicator()<tolerance
        
        def calibrated(self,tolerance):
                return self.position_calibrated(tolerance) and self.orientation_calibrated(tolerance) and self.k_calibrated(tolerance)
        
        def project(self,point):
                return project_wide_fov(point,np.array([1,0,0,0]),np.array([0,0,0]),self.position,self.orientation_q.elements,self.k,self.max_pix_r,self.distortion_coefficient)
        
        def __str__(self) -> str:
                return "WideFoVCamera\nframe_shape = "+str(self.frame_shape)+"\nposition = "+str(self.position)+"\norientation_q = "+str(self.orientation_q)+"\nfov_deg = "+str(self.fov_deg)+"\ndistortion_coefficient = "+str(self.distortion_coefficient)+"\ndistortion_coefficient_cov = "+str(self.distortion_coefficient_cov)+"\nposition_cov = "+str(self.position_cov)+"\norientation_q_cov = "+str(self.orientation_q_cov)+"\nk = "+str(self.k)+"\nk_cov = "+str(self.k_cov)+"\nmax_pix_r = "+str(self.max_pix_r)+"\nled_measurement_cov = "+str(self.led_measurement_cov)
        
def project_wide_fov(led_pos,drone_orien,drone_pos,cam_pos,cam_orien,cam_k,max_pix,distortion_coefficient):
        '''
        project a point from world frame to camera frame (for wide fov camera) -> barrel distortion
        '''
        projection = project(led_pos,drone_orien,drone_pos,cam_pos,cam_orien,cam_k)
        #destination radius
        r = np.linalg.norm(projection)/max_pix
        return projection/(1+distortion_coefficient*r**2)

def project(led_pos,drone_orien,drone_pos,cam_pos,cam_orien,cam_k):
        '''
        project a point from world frame to camera frame
        '''
        qdcm = quat2dcm(drone_orien[0],-drone_orien[1:4])
        res = qdcm@led_pos
        res1 = res + drone_pos - cam_pos
        qdcm2 = quat2dcm(cam_orien[0],cam_orien[1:4])
        res2 = qdcm2@res1
        if res2[2] < 0:
                return np.array([0,0])
        res3 = res2[0:2]*cam_k/res2[2]
        return res3

env = environment()
criticalState = CriticalState()
imu = MPU9250()
my_ekf = ekf.Ekf()
leds = []
cams = []
Q = None


def transition_function(X,U,dt_imu,g):
        '''
        transition function
        '''
        q0,qvec = X[0],X[1:4]
        xyz = X[4:7]
        vxyz = X[7:10]
        gbxyz = X[10:13]
        abxyz = X[13:16]
        gyro_o = X[16:25].reshape(3,3)
        acc_o = X[25:34].reshape(3,3)
        gyro = U[0:3]
        acc = U[3:6]
        
        #correction of gyro
        gyro = gyro_o@(gyro+gbxyz)

        #integrate the orientation
        q_ = Quaternion(q0,qvec[0],qvec[1],qvec[2])
        q_.integrate(gyro,dt_imu)
        q_ = q_.elements
        
        #correction of acc
        acc = acc_o@(acc+abxyz)
        
        #rotate the acceleration to the world frame
        qdcm = quat2dcm(q0,-qvec)
        ae = qdcm@acc
        ae[2] = ae[2] + g
        
        #integrate the linear velocity
        vxyz_ = vxyz + ae*dt_imu

        
        #integrate the position
        xyz_ = xyz + vxyz_*dt_imu
        
                
        newX = X.copy()
        newX[0:4] = q_[0:4]
        newX[4:7] = xyz_
        newX[7:10] = vxyz_
        
        return newX

def hcmln(X, m, n):
        drone_orien = X[0:4].copy()
        drone_pos = X[4:7].copy()
        led_pos = leds[n].mounting_position.copy()
        cam_pos = X[m*9+34:m*8+37].copy()
        cam_orien = X[m*9+37:m*8+41].copy()
        k = X[41+m*9].copy()
        alpha = X[42+m*9].copy()
        return project_wide_fov(led_pos,drone_orien,drone_pos,cam_pos,cam_orien,k,cams[m].max_pix_r,alpha)
        
        
def calib2X(Q, x, P, criState, imu, cams):
        '''
        link object to Q, x and P
        '''
        if (Q is None):
                Q = np.zeros((6,6))
                
        Q[0:3,0:3] = imu.gyr_noise
        Q[3:6,3:6] = imu.acc_noise
        

        if (x is None):
                x = np.zeros(34+9*len(cams))
                
        x[0:4] = criState.orientation.elements
        x[4:7] = criState.position
        x[7:10] = criState.velocity
        x[10:13] = imu.gyr_bias_co
        x[13:16] = imu.acc_bias_co
        x[16:25] = imu.gyr_ortho_co.reshape(9)
        x[25:34] = imu.acc_ortho_co.reshape(9)

        for m in range(len(cams)):
                x[34+m*9:37+m*9] = cams[m].position
                x[37+m*9:41+m*9] = cams[m].orientation_q.elements
                x[41+m*9] = cams[m].k
                x[42+m*9] = cams[m].distortion_coefficient
        
        if (P is None):
                P = np.zeros((x.shape[0],x.shape[0]))
                
        P[0:4,0:4] = criState.orientation_cov
        P[4:7,4:7] = criState.position_cov
        P[7:10,7:10] = criState.velocity_cov
        P[10:13,10:13] = imu.gyr_bias_cov
        P[13:16,13:16] = imu.acc_bias_cov
        P[16:25,16:25] = imu.gyr_ortho_cov
        P[25:34,25:34] = imu.acc_ortho_cov
        
        for m in range(len(cams)):
                P[34+m*9:37+m*9,34+m*9:37+m*9] = cams[m].position_cov
                P[37+m*9:41+m*9,37+m*9:41+m*9] = cams[m].orientation_q_cov
                P[41+m*9,41+m*9] = cams[m].k_cov
                P[42+m*9,42+m*9] = cams[m].distortion_coefficient_cov
                        
        return Q,x,P

def X2calib(X,P, criState, imu, cams):
        '''
        link state vector to object
        '''
        criState.orientation = Quaternion(X[0],X[1],X[2],X[3])
        criState.position = X[4:7]
        criState.velocity = X[7:10]
        imu.gyr_bias_co = X[10:13]
        imu.acc_bias_co = X[13:16]
        imu.gyr_ortho_co = X[16:25].reshape(3,3)
        imu.acc_ortho_co = X[25:34].reshape(3,3)
        
        for m in range(len(cams)):
                cams[m].position = X[34+m*9:37+m*9]
                cams[m].orientation_q = Quaternion(X[37+m*9],X[38+m*9],X[39+m*9],X[40+m*9])
                cams[m].orientation_rpy = quat2angle(X[37+m*9],np.array(X[38+m*9:41+m*9]),rotation_sequence='ZYX',output_unit='deg') 
                cams[m].k = X[41+m*9]
                cams[m].distortion_coefficient = X[42+m*9]
               
        criState.orientation_cov = P[0:4,0:4]
        criState.position_cov = P[4:7,4:7]
        criState.velocity_cov = P[7:10,7:10]
        imu.gyr_bias_cov = P[10:13,10:13]
        imu.acc_bias_cov = P[13:16,13:16]
        imu.gyr_ortho_cov = P[16:25,16:25]
        imu.acc_ortho_cov = P[25:34,25:34]
        
        for m in range(len(cams)):
                cams[m].position_cov = P[34+m*9:37+m*9,34+m*9:37+m*9]
                cams[m].orientation_q_cov = P[37+m*9:41+m*9,37+m*9:41+m*9]
                cams[m].k_cov = P[41+m*9,41+m*9]
                cams[m].distortion_coefficient_cov = P[42+m*9,42+m*9]       
   
        return criState, imu, cams
      
def init():
        global Q, my_ekf
        my_ekf = ekf.Ekf(34+9*len(cams))
        Q, my_ekf.x, my_ekf.P = calib2X(Q, my_ekf.x, my_ekf.P, criticalState, imu, cams)

def initAttitudeUsingAcc():
        '''
        compute the initial orientation using the accelerometer
        '''
        pitch = -np.arctan2(-imu.new_acc_sample[0], np.sqrt(imu.new_acc_sample[1]**2 + imu.new_acc_sample[2]**2))
        roll = np.arctan2(-imu.new_acc_sample[1], -imu.new_acc_sample[2])
        # print("pitch, roll =", pitch,roll)
        q0, qvec = angle2quat(0, pitch, roll, input_unit='rad')
        criticalState.orientation = Quaternion(q0,qvec[0],qvec[1],qvec[2])
        criticalState.setOriCov(20/180.0*3.14)
        calib2X(Q, my_ekf.x, my_ekf.P, criticalState, imu, cams)

def predict():
        '''
        predict the state
        '''
        global criticalState, imu
                
        if imu.new_gyr_sample is None:
                return
        
        if imu.prev_time is None:
                imu.prev_time = imu.time
                return
        
        u = np.concatenate((imu.new_gyr_sample,imu.new_acc_sample))
        f = lambda x,u: transition_function(x,u,imu.time - imu.prev_time,env.gravity)
        my_ekf.predict(f,u,Q)
        imu.prev_time = imu.time
        imu.new_gyr_sample = None
        imu.new_acc_sample = None

        X2calib(my_ekf.x, my_ekf.P, criticalState, imu, cams)


    
def find_best_perm(cam_m, maxMahalanobis=10000):
        best_perm = None
       
        avr_d_min=100000
        if  len(cams[cam_m].fresh_led_measurements) < len(leds):
                for perm in permutations(range(len(leds)), len(cams[cam_m].fresh_led_measurements)):
                        d = 0
                        count = 0
                        best_sub_j = [None]*len(leds)
                        for i in range(len(perm)):
                                h = lambda x: hcmln(x,cam_m,perm[i])
                                sub_d = np.inf
                                min_j = len(cams[cam_m].fresh_led_measurements[i])
                                for j in range(len(cams[cam_m].fresh_led_measurements[i])):
                                        tmp = my_ekf.mahalanobis(h ,cams[cam_m].fresh_led_measurements[i][j],cams[0].led_measurement_cov) 

                                        if tmp < sub_d:
                                                sub_d = tmp
                                                min_j = j
                                        
                                if sub_d < 20:
                                        d += sub_d
                                        count += 1
                                        best_sub_j[perm[i]] = min_j

                        if count != 0:
                                d = d/count
                                if d < avr_d_min:
                                        avr_d_min = d
                                        perm2 = [0]*(max(perm)+1)
                                        for i in range(len(perm2)):
                                                perm2[i] = None
                                        for i in range(len(perm)):
                                                perm2[perm[i]] = i
                                        
                                        perm2 = tuple(perm2)  
                                        
                                        best_perm = (perm2,best_sub_j)
        else:
                # best_perm = 
                for perm in permutations(range(len(cams[cam_m].fresh_led_measurements)), len(leds)):
                        print("perm = ",perm)
                        d = 0
                        count = 0
                        best_sub_j = []
                        for i in range(len(perm)):
                                h = lambda x: hcmln(x,cam_m,i)
                                sub_d = 100000
                                min_j = len(cams[cam_m].fresh_led_measurements[perm[i]])
                                for j in range(len(cams[cam_m].fresh_led_measurements[perm[i]])):
                                        tmp = my_ekf.mahalanobis(h ,cams[cam_m].fresh_led_measurements[perm[i]][j],cams[0].led_measurement_cov) 

                                        if tmp < sub_d:
                                                sub_d = tmp
                                                min_j = j
                                
                                if sub_d < maxMahalanobis:
                                        d += sub_d
                                        count += 1
                                        best_sub_j.append(min_j)
                                else:
                                        best_sub_j.append(None)
                        if count != 0:
                                d = d/count
                                if d < avr_d_min:
                                        avr_d_min = d
                                        best_perm = (perm,best_sub_j)
                                        
        return best_perm

    
from itertools import permutations

def update():
        '''
        update the state using measurements
        '''
        global criticalState, imu, cams
        for m in range(len(cams)):
                print("reshresh" , m)
                cams[m].predicted_led_measurements = []
                cams[m].predicted_led_measurements_cov = []
                if cams[m].fresh_led_measurements is not None and cams[m].fresh_led_measurements != []:
                        best_perm = find_best_perm(m)
                        print("best_perm = ",best_perm)
                        #update the state
                        if best_perm is not None:
                                if best_perm[0] is not None:
                                        for i in range(len(best_perm[0])):
                                                if best_perm[1][i] is not None:
                                                        h = lambda x: hcmln(x,m,i)
                                                        new_h,new_S = my_ekf.update(h,cams[m].fresh_led_measurements[best_perm[0][i]][best_perm[1][i]],cams[m].led_measurement_cov)
                                                        cams[m].predicted_led_measurements.append(new_h)
                                                        cams[m].predicted_led_measurements_cov.append(new_S)
                                                        print("new_h = ",new_h)
                                                        print("new_S = ",new_S)
                                                        
                        cams[m].last_led_measurements = cams[m].fresh_led_measurements
                        cams[m].fresh_led_measurements = None

         
        criticalState, imu, cams = X2calib(my_ekf.x, my_ekf.P, criticalState, imu, cams)                
if __name__ == "__main__":
        import cv2
        imu.setAccNoise(0.1)
        imu.setGyrNoise(0.1)
        cams.append(WideFoVCamera(0,np.array([0,0,0]),np.array([0,0,0]),120,0.16))
        cams[0].setPosCov([0,0,0])
        cams[0].setOriCov([0,0,0])
        leds.append(led(np.array([0,0,0])))
        init()
        imu.time = 0.1
        imu.prev_time = 0
        imu.new_gyr_sample = np.array([0,0,0])
        imu.new_acc_sample = np.array([0,0,0])
        predict()
        cams[0].fresh_led_measurements = [[np.array([0,0])]]
        update()
        imu.time = 0.2
        imu.new_gyr_sample = np.array([0,0,0])
        imu.new_acc_sample = np.array([0,0,-20])
        predict()
        cams[0].fresh_led_measurements = [[np.array([0,0])]]
        update()     

        print(cams[0].predicted_led_measurements)
        print(cams[0].predicted_led_measurements_cov)
        # Libérer les ressources et fermer les fenêtres    
        while True:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                cams[0].read()
                if cams[0].frame is None:
                        continue
                cv2.imshow('frame',cams[0].buid_desription_frame())