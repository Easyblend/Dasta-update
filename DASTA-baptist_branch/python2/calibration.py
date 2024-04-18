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
postion(xyz)                    m           0:2
velocity(xyz)                   m/s         3:5
orientation(quaternion)                     6:9
gyr_bias_correction(xyz)        rad/s       10:12
acc_bias_correction(xyz)        m/s²        13:15
gyr_ortho_correction(3*3)                   16:24
acc_ortho_correction(3*3)                   25:33

.............camera parameters................
cam_0_pos(xyz)                  m           34:36
cam_0_ori(quaternion)                       37:40
cam_0_k                         pixel       41
                 ...
cam_m_pos(xyz)                  m           34+8*m:36+8*m
cam_m_ori(quaternion)                       37+8*m:40+8*m
cam_m_k                         pixel       41+8*m


___________COMMANDS VECTOR_____________________

        State                   unit        index
gyroscopes(xyz)                 rad/s       0:2
accelerometers(xyz)             m/s²        3:5


___________SENSOR VECTORS_____________________

        State                   unit       index
beac1_cam1(uv)                  pixel       0:1

'''           
class CriticalState:
        def __init__(self,position = np.array([0,0,0]),velocity = np.array([0,0,0]), orientation = Quaternion(1,0,0,0)):
                self.position = position
                self.velocity = velocity
                self.orientation = orientation
                
                self.setPosCov(0)
                self.setVelCov(0)
                self.setOriCov(0)
                
                
        def setPosCov(self, pos_accur):
                self.position_cov = np.eye(3)*pos_accur**2
        
        def setVelCov(self, vel_accur):
                self.velocity_cov = np.eye(3)*vel_accur**2
        
        def setOriCov(self, ori_accur):
                self.orientation_cov = np.eye(4)*ori_accur**2
                
        
        def position_calibrated(self, tolerance=0.1):
                return np.all(np.sqrt(np.diag(self.position_cov))<tolerance)
        
        def velocity_calibrated(self, tolerance=0.05):
                return np.all(np.sqrt(np.diag(self.velocity_cov))<tolerance)
        
        def orientation_calibrated(self, tolerance=0.01):
                return np.all(np.sqrt(np.diag(self.orientation_cov))<tolerance)
        
        def calibrated(self):
                return self.position_calibrated() and self.velocity_calibrated() and self.orientation_calibrated()
                
        def __str__(self) -> str:
                return "criticalState\nposition = "+str(self.position)+"\nvelocity = "+str(self.velocity)+"\norientation = "+str(self.orientation)+"\nposition_cov = "+str(self.position_cov)+"\nvelocity_cov = "+str(self.velocity_cov)+"\norientation_cov = "+str(self.orientation_cov)
                
                
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
                self.setAccOrthoCov(acc_cross_tolerance,acc_cross_tolerance)
                
                self.gyr_bias_co = np.zeros(3)
                self.acc_bias_co = np.zeros(3)
                
                self.gyr_ortho_co = np.eye(3)
                self.acc_ortho_co = np.eye(3)
                
                
                

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
                
                
                
        def gyr_bias_calibrated(self, tolerance=0.0001):
                return np.all(np.sqrt(np.diag(self.gyr_bias_cov))<tolerance)
        
        def acc_bias_calibrated(self, tolerance=0.005):
                return np.all(np.sqrt(np.diag(self.acc_bias_cov))<tolerance)
        
        def gyr_ortho_calibrated(self, tolerance=0.005):
                return np.all(np.sqrt(np.diag(self.gyr_ortho_cov))<tolerance)
        
        def acc_ortho_calibrated(self, tolerance=0.005):
                return np.all(np.sqrt(np.diag(self.acc_ortho_cov))<tolerance)
        
        def calibrated(self):
                return self.gyr_bias_calibrated() and self.acc_bias_calibrated() and self.gyr_ortho_calibrated() and self.acc_ortho_calibrated()
        
        def __str__(self) -> str:
                return "MPU9250\ngyr_noise = "+str(self.gyr_noise)+"\nacc_noise = "+str(self.acc_noise)+"\nacc_bias_co = "+str(self.acc_bias_co)+"\ngyr_bias_co = "+str(self.gyr_bias_co)+"\nacc_ortho_co = "+str(self.acc_ortho_co)+"\ngyr_ortho_co = "+str(self.gyr_ortho_co)+"\nacc_bias_cov = "+str(self.acc_bias_cov)+"\ngyr_bias_cov = "+str(self.gyr_bias_cov)+"\nacc_ortho_cov = "+str(self.acc_ortho_cov)+"\ngyr_ortho_cov = "+str(self.gyr_ortho_cov)
                           

class led:
        def __init__(self,mounting_position = np.array([0,0,0])):
                self.mounting_position = mounting_position
                
        def __str__(self) -> str:
                return "led\nmounting_position = "+str(self.mounting_position)
                
class camera:
        def __init__(self,position=np.array([0,0,0]), orientation=Quaternion(1,0,0,0), k=1,  pos_accur=0, ori_accur=0, k_accur=0, noise=1):
                self.position = position
                self.orientation = orientation
                self.k = k
                
                self.setPosCov(pos_accur)
                self.setOriCov(ori_accur)
                self.setKCov(k_accur)

                self.noise = noise
                self.fresh_led_measurements = None
                self.last_led_measurements = None
                self.led_measurement_cov = np.eye(2)*noise**2
                
                self.imageShape = None
                
        def setOri(self, Rzyx, input_unit = 'deg'):
                q0, qvec = angle2quat(Rzyx[0],Rzyx[1],Rzyx[2], input_unit=input_unit)
                self.orientation = Quaternion(q0,qvec[0],qvec[1],qvec[2])
                
        def getRzyx(self, output_unit = 'deg'):
                q0, qvec = self.orientation.elements[0], self.orientation.elements[1:4]
                return list(quat2angle(q0, qvec, output_unit=output_unit))

        def setPosCov(self, pos_accur):
                self.pos_cov = np.eye(3)*pos_accur**2
        
        def setOriCov(self, ori_accur):
                self.ori_cov = np.eye(4)*ori_accur**2
                
        def setKCov(self, k_accur):
                self.k_cov = k_accur**2
                
        def position_calibrated(self, tolerance=0.1):
                return np.all(np.sqrt(np.diag(self.pos_cov))<tolerance)
        
        def orientation_calibrated(self, tolerance=0.01):
                return np.all(np.sqrt(np.diag(self.ori_cov))<tolerance)
        
        def k_calibrated(self, tolerance=1):
                return np.sqrt(self.k_cov)<tolerance
        
        def calibrated(self):
                return self.position_calibrated() and self.orientation_calibrated() and self.k_calibrated()
                
        def __str__(self) -> str:
                return "camera\nposition = "+str(self.position)+"\norientation = "+str(self.orientation)+"\nk = "+str(self.k)+"\npos_cov = "+str(self.pos_cov)+"\nori_cov = "+str(self.ori_cov)+"\nk_cov = "+str(self.k_cov)+"\nnoise = "+str(self.noise)+"\nled_measurements = "+str(self.last_led_measurements)+"\nled_measurement_cov = "+str(self.led_measurement_cov)

class environment:
        def __init__(self, gravity=9.81):
                self.gravity = gravity
                
        def __str__(self) -> str:
                return "environment\ngravity = "+str(self.gravity)
                

#instanciation of the system
env = environment()
criticalState = CriticalState()
imu = MPU9250()        
leds = [led() for i in range(2)]
cams = [camera() for i in range(1)]

my_ekf = ekf.Ekf()
Q = None




def transition_function(X,U,dt_imu,g):
        '''
        transition function
        '''
        xyz = X[0:3]
        vxyz = X[3:6]
        q0,qvec = X[6],X[7:10]
        gbxyz = X[10:13]
        abxyz = X[13:16]
        gyro_o = X[16:25].reshape(3,3)
        acc_o = X[25:34].reshape(3,3)
        
        gyro = U[0:3]
        acc= U[3:6]
        
        #correction of gyro and acc
        gyro = gyro_o@(gyro+gbxyz)

        acc = acc_o@(acc+abxyz)    
        # integration of linear velocities
        xyz_ = xyz + vxyz*dt_imu
        
        # integration of linear accelerations
        # print("acc = ",acc)
        qdcm = quat2dcm(q0,-qvec)
        ae = qdcm@acc
        ae[2] = ae[2] + g
        vxyz_ = vxyz + ae*dt_imu
        q_ = Quaternion(q0,qvec[0],qvec[1],qvec[2])
        q_.integrate(gyro,dt_imu)
        q_ = q_.elements
        
        newX = X.copy()
        newX[0:3] = xyz_
        newX[3:6] = vxyz_
        newX[6:10] = q_[0:4]
        
        return newX


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

def hcmln(X, m, n):
        drone_orien = X[6:10].copy()
        drone_pos = X[0:3].copy()
        led_pos = leds[n].mounting_position.copy()
        cam_pos = X[m*8+34:m*8+37].copy()
        cam_orien = X[m*8+37:m*8+41].copy()
        k = X[41+m*8].copy()
        return project(led_pos,drone_orien,drone_pos,cam_pos,cam_orien,k)


def calib2X(Q, x, P, criState, imu, cams):
        '''
        link object to Q, x and P
        '''
        if (Q is None):
                Q = np.zeros((6,6))
                
        Q[0:3,0:3] = imu.gyr_noise
        Q[3:6,3:6] = imu.acc_noise
        

        if (x is None):
                x = np.zeros(34+8*len(cams))
                
        x[0:3] = criState.position
        x[3:6] = criState.velocity
        x[6:10] = criState.orientation.elements
        x[10:13] = imu.gyr_bias_co
        x[13:16] = imu.acc_bias_co
        x[16:25] = imu.gyr_ortho_co.reshape(9)
        x[25:34] = imu.acc_ortho_co.reshape(9)

        for i in range(len(cams)):
                x[34+8*i:37+8*i] = cams[i].position
                x[37+8*i:41+8*i] = cams[i].orientation.elements
                x[41+8*i] = cams[i].k
                
        if (P is None):
                P = np.zeros((x.shape[0],x.shape[0]))
                
        P[0:3,0:3] = criState.position_cov
        P[3:6,3:6] = criState.velocity_cov
        P[6:10,6:10] = criState.orientation_cov
        P[10:13,10:13] = imu.gyr_bias_cov
        P[13:16,13:16] = imu.acc_bias_cov
        P[16:25,16:25] = imu.gyr_ortho_cov
        P[25:34,25:34] = imu.acc_ortho_cov
        for i in range(len(cams)):
                P[34+8*i:37+8*i,34+8*i:37+8*i] = cams[i].pos_cov
                P[37+8*i:41+8*i,37+8*i:41+8*i] = cams[i].ori_cov
                P[41+8*i,41+8*i] = cams[i].k_cov
                
        return Q,x,P

def X2calib(X,P, criState, imu, cams):
        '''
        link state vector to object
        '''
        criState.position = X[0:3]
        criState.velocity = X[3:6]
        criState.orientation = Quaternion(X[6],X[7],X[8],X[9])
        imu.gyr_bias_co = X[10:13]
        imu.acc_bias_co = X[13:16]
        imu.gyr_ortho_co = X[16:25].reshape(3,3)
        imu.acc_ortho_co = X[25:34].reshape(3,3)
        
        for i in range(len(cams)):
                cams[i].position = X[34+8*i:37+8*i]
                cams[i].orientation = Quaternion(X[37+8*i:41+8*i])
                cams[i].k = X[41+8*i]
                
        criState.position_cov = P[0:3,0:3]
        criState.velocity_cov = P[3:6,3:6]
        criState.orientation_cov = P[6:10,6:10]
        
        for i in range(len(cams)):
                cams[i].pos_cov = P[34+8*i:37+8*i,34+8*i:37+8*i]
                cams[i].ori_cov = P[37+8*i:41+8*i,37+8*i:41+8*i]
                cams[i].k_cov = P[41+8*i,41+8*i]
                
        return criState, imu, cams
      
def init():
        global Q, my_ekf
        my_ekf = ekf.Ekf(34+8*len(cams))
        Q, my_ekf.x, my_ekf.P = calib2X(Q, my_ekf.x, my_ekf.P, criticalState, imu, cams)


def predict():
        '''
        predict the state
        '''
        global criticalState, imu, cams
                
        if imu.new_gyr_sample is None or imu.new_acc_sample is None:
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

        criticalState, imu, cams = X2calib(my_ekf.x, my_ekf.P, criticalState, imu, cams)


def generate_measurements(cams, leds, my_ekf, hcmln, chance1 = 10 , chance2 = 3, chance3 = 20, noise = 0.5):
        '''
        generate measurements
        '''
        for m in range(len(cams)):
                cams[m].fresh_led_measurements = []
                for n in range(len(leds)):
                        xx = my_ekf.x.copy()
                        xx[0:3] = 0
                        
                        z = hcmln(xx,m,n)
                        z = z + np.random.normal(0,noise,2)
                        
                        #une chance sur 10 de ne pas avoir de mesure de la led
                        new_measure = []
                        if np.random.randint(0,chance1) != 0:
                                new_measure.append(z)
                        
                        #une chance sur 3 d'avoir une deuxieme mesure aberrante
                        if np.random.randint(0,chance2) == 0:
                                new_measure.append(z + np.random.normal(0,10,2))
                        
                        # les mesures ne seront pas dans l'ordre
                        np.random.shuffle(new_measure)
                        if len(new_measure) != 0:
                                cams[m].fresh_led_measurements.append(new_measure)
                        
                #une chance sur 20 de mesurer autre chose
                if np.random.randint(0,chance3) == 0:
                        cams[m].fresh_led_measurements.append([np.random.normal(0,100,2)])      
                        
                # les mesures ne seront pas dans l'ordre
                np.random.shuffle(cams[m].fresh_led_measurements)
                
def find_best_perm(cam_m, maxMahalanobis=1000):
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
                                        # perm = (1,) means that the first shape corresponds to the second led
                                        # perm = (1,0) means that the first shape corresponds to the second led and the second shape corresponds to the first led
                                        # we need to reverse the permutation (1,) in order to have (None,0) means that the first shape corresponds to the second led or that the fist shape do not exist and the second shape corresponds to the first led
                                        # we need to reverse the permutation (1,0) in order to have (1,0) means that the first led corresponds to the second shape and the second led corresponds to the first shape
                                        perm2 = [0]*(max(perm)+1)
                                        for i in range(len(perm2)):
                                                perm2[i] = None
                                        for i in range(len(perm)):
                                                perm2[perm[i]] = i
                                        
                                        perm2 = tuple(perm2)  
                                        
                                        best_perm = (perm2,best_sub_j)
        else:
                
                for perm in permutations(range(len(cams[cam_m].fresh_led_measurements)), len(leds)):
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

def update(force_center = None):
        '''
        update the state using measurements
        '''
        global criticalState, imu, cams
        
        if force_center is not None:
                generate_measurements(cams, leds, my_ekf, hcmln, 1000, 1000, 1000, 0)   
   
        
        for m in range(len(cams)):
                if cams[m].fresh_led_measurements is not None and cams[m].fresh_led_measurements != []:
                        best_perm = find_best_perm(m)
                                                        
                        #update the state
                        if best_perm is not None:
                                if best_perm[0] is not None:
                                        for i in range(len(best_perm[0])):
                                                if best_perm[1][i] is not None:
                                                        h = lambda x: hcmln(x,m,i)
                                                        my_ekf.update(h,cams[m].fresh_led_measurements[best_perm[0][i]][best_perm[1][i]],cams[m].led_measurement_cov)
        
                        cams[m].last_led_measurements = cams[m].fresh_led_measurements
                        cams[m].fresh_led_measurements = None

         
        criticalState, imu, cams = X2calib(my_ekf.x, my_ekf.P, criticalState, imu, cams)                
        
        
if __name__ == "__main__":
        # cams[0].se 
        q0,qvec = angle2quat(135,-36,0)       
        cams[0].orientation = Quaternion(q0,qvec[0],qvec[1],qvec[2])
        print("cam0 = ",cams[0].orientation.elements)
        