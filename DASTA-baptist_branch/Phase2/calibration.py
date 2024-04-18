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

        State                   unit        index
orientation(quaternion)                     0:3
gyr_bias_correction(xyz)        rad/s       4:6
acc_bias_correction(xyz)        m/s²        7:9
gyr_ortho_correction(3*3)                   10:18
acc_ortho_correction(3*3)                   19:27

___________COMMANDS VECTOR_____________________

        State                   unit        index
gyroscopes(xyz)                 rad/s       0:2


___________SENSOR VECTORS_____________________

        State                   unit       index
accelerometers(xyz)             m/s²        3:5

'''           
class CriticalState:
        def __init__(self,orientation = Quaternion(1,0,0,0)):
                self.orientation = orientation          
                self.setOriCov(0)
                
                        
        def setOriCov(self, ori_accur):
                self.orientation_cov = np.eye(4)*ori_accur**2
                
        
        def orientation_calibrated(self, tolerance=0.01):
                return np.all(np.sqrt(np.diag(self.orientation_cov))<tolerance)
        
                
        def __str__(self) -> str:
                return "criticalState\norientation = "+str(self.orientation)+"\norientation_cov = "+str(self.orientation_cov)
                
                
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
                
                
def project(led_pos,drone_orien,drone_pos,cam_pos,cam_orien,cam_k):
        '''
        project a point from world frame to camera frame
        '''
        qdcm = quat2dcm(drone_orien[0],-drone_orien[1:4])
        res = qdcm@led_pos
        res1 = res + drone_pos - cam_pos
        qdcm2 = quat2dcm(cam_orien[0],cam_orien[1:4])
        res2 = qdcm2@res1
        res3 = res2[0:2]*cam_k/res2[2]
        return res3

env = environment()
criticalState = CriticalState()
imu = MPU9250()
my_ekf = ekf.Ekf()
Q = None




def transition_function(X,U,dt_imu):
        '''
        transition function
        '''
        q0,qvec = X[0],X[1:4]
        gbxyz = X[4:7]
        gyro_o = X[10:19].reshape(3,3)        
        gyro = U[0:4]
        
        #correction of gyro
        gyro = gyro_o@(gyro+gbxyz)

        
        q_ = Quaternion(q0,qvec[0],qvec[1],qvec[2])
        q_.integrate(gyro,dt_imu)
        q_ = q_.elements
        
        newX = X.copy()
        newX[0:4] = q_[0:4]
        
        return newX

def h(X):
        '''
        measurement function
        '''
        global env
        q0,qvec = X[0],X[1:4]
        q_ = Quaternion(q0,-qvec[0],-qvec[1],-qvec[2])
        acc = q_.rotate([0,0,-env.gravity])
        #apply bias and ortho correction
        o_inv = X[19:28].reshape(3,3)
        o_inv = np.linalg.inv(o_inv)
        acc = o_inv@acc-X[7:10]
        return acc

def calib2X(Q, x, P, criState, imu):
        '''
        link object to Q, x and P
        '''
        if (Q is None):
                Q = np.zeros((3,3))
                
        Q[0:4,0:4] = imu.gyr_noise
        

        if (x is None):
                x = np.zeros(28)
                
        x[0:4] = criState.orientation.elements
        x[4:7] = imu.gyr_bias_co
        x[7:10] = imu.acc_bias_co
        x[10:19] = imu.gyr_ortho_co.reshape(9)
        x[19:28] = imu.acc_ortho_co.reshape(9)

        if (P is None):
                P = np.zeros((x.shape[0],x.shape[0]))
                
        P[0:4,0:4] = criState.orientation_cov
        P[4:7,4:7] = imu.gyr_bias_cov
        P[7:10,7:10] = imu.acc_bias_cov
        P[10:19,10:19] = imu.gyr_ortho_cov
        P[19:28,19:28] = imu.acc_ortho_cov
                        
        return Q,x,P

def X2calib(X,P, criState, imu):
        '''
        link state vector to object
        '''
        criState.orientation = Quaternion(X[0],X[1],X[2],X[3])
        imu.gyr_bias_co = X[4:7]
        imu.acc_bias_co = X[7:10]
        imu.gyr_ortho_co = X[10:19].reshape(3,3)
        imu.acc_ortho_co = X[19:28].reshape(3,3)
                        
        criState.orientation_cov = P[0:4,0:4]
        imu.gyr_bias_cov = P[4:7,4:7]
        imu.acc_bias_cov = P[7:10,7:10]
        imu.gyr_ortho_cov = P[10:19,10:19]
        imu.acc_ortho_cov = P[19:28,19:28]
        
   
        return criState, imu
      
def init():
        global Q, my_ekf
        my_ekf = ekf.Ekf(28)
        Q, my_ekf.x, my_ekf.P = calib2X(Q, my_ekf.x, my_ekf.P, criticalState, imu)

def initAttitudeUsingAcc():
        '''
        compute the initial orientation using the accelerometer
        '''
        pitch = np.arctan2(-imu.new_acc_sample[0], np.sqrt(imu.new_acc_sample[1]**2 + imu.new_acc_sample[2]**2))
        roll = np.arctan2(-imu.new_acc_sample[1], -imu.new_acc_sample[2])
        # print("pitch, roll =", pitch,roll)
        q0, qvec = angle2quat(0, pitch, roll, input_unit='rad')
        criticalState.orientation = Quaternion(q0,qvec[0],qvec[1],qvec[2])
        criticalState.setOriCov(20/180.0*3.14)
        calib2X(Q, my_ekf.x, my_ekf.P, criticalState, imu)

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
        
        u = imu.new_gyr_sample
        f = lambda x,u: transition_function(x,u,imu.time - imu.prev_time)
        my_ekf.predict(f,u,Q)
        imu.prev_time = imu.time
        imu.new_gyr_sample = None

        criticalState, imu = X2calib(my_ekf.x, my_ekf.P, criticalState, imu)


def update():
        '''
        update the state using measurements
        '''
        global criticalState, imu
        

        # my_ekf.update(h,cams[m].fresh_led_measurements[best_perm[0][i]][best_perm[1][i]],cams[m].led_measurement_cov)
        
        my_ekf.update(h,imu.new_acc_sample,imu.acc_noise)
        criticalState, imu = X2calib(my_ekf.x, my_ekf.P, criticalState, imu)                
        
        
if __name__ == "__main__":
        init()
        imu.new_gyr_sample = np.array([0.1,0.2,0.3])
        imu.time = 0.0
        print(criticalState)
        predict()
        print(criticalState)
        imu.time += 0.1
        predict()
        print(criticalState)
        print(h(my_ekf.x))
        