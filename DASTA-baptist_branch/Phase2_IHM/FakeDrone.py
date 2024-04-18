import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
import time
import ekf

class FakePropeller:
    def __init__(self):
        self.cw = 1
        self.thrust_coeff = 0.0011711 + np.random.normal(0,0.0001)
        self.drag_torque_coeff = -0.00001989653 + np.random.normal(0,0.000005)
        self.thrust = 0
        self.drag_torque = 0
        
    def update(self, w):
        self.thrust = self.thrust_coeff*w
        self.drag_torque = self.drag_torque_coeff*w * self.cw
        return self.thrust, self.drag_torque

class FakeEngine:
    def __init__(self):
        self.propeller = FakePropeller()
        self.cw = 1
        self.speedNoise = 10
        self.max_speed = 2513 #rad/s
        self.time_constant = 0.15
        self.speed = 0
        self.position = np.array([0,0,0])
        self.in_field_velocity = np.array([0,0,0])
        self.u = 0
        
    def update(self, u, air_rotation, time_step):
        self.u = u
        alpha = np.exp(-time_step/self.time_constant)
        self.speed *= alpha
        self.speed += (1-alpha)*u*self.max_speed
        return self.propeller.update(self.speed-air_rotation+np.random.normal(0,self.speedNoise))

class FakeIMU:
    def __init__(self):
        self.acc = np.array([0,0,0])
        self.gyr = np.array([0,0,0])
        self.accNoise = 0.1
        self.gyrNoise = 0.0017
        self.accBias = np.random.normal(0,0.7848,3)
        self.gyrBias = np.random.normal(0,0.03,3)
        self.accOrtho = np.random.normal(0,0.01,[3,3]) + np.eye(3)
        self.gyrOrtho = np.random.normal(0,0.01,[3,3]) + np.eye(3)
        
    def update(self, true_acc, true_gyr, time_step):
        self.acc = true_acc.dot(self.accOrtho) + self.accBias + np.random.normal(0,self.accNoise,3)
        self.gyr = true_gyr.dot(self.gyrOrtho) + self.gyrBias + np.random.normal(0,self.gyrNoise,3)
        return self.acc, self.gyr
    
class Pid:
    def __init__(self, kp, ki, kd, max_output, min_output, max_integral=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.max_integral = max_integral
        self.integral = 0
        self.prev_error = 0
    
    def update(self, error, time_step):
        self.integral += error*time_step
        if self.integral*self.ki > self.max_integral:
            self.integral = self.max_integral/self.ki
        elif self.integral*self.ki < -self.max_integral:
            self.integral = -self.max_integral/self.ki
            
            
        derivative = (error-self.prev_error)/time_step
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error
        if output > self.max_output:
            output = self.max_output
        elif output < self.min_output:
            output = self.min_output
        return output
    
class FakeState:
    def __init__(self):
        self.position = np.array([0.0,0.0,0.0])
        self.orientation = Quaternion()
        self.speeds = np.array([0.0,0.0,0.0])
        self.omega = np.array([0.0,0.0,0.0])
        self.omega_prime = np.array([0.0,0.0,0.0])
        self.acceleration = np.array([0.0,0.0,0.0])
        
class FakeEstimator:
    def __init__(self):
        self.ekf = ekf.Ekf(4)
        self.ekf.x = np.array([1,0,0,0])
        self.ekf.P = np.eye(4)*0.1
        self.orientation = Quaternion()
        self.gyr_cov = np.eye(3)*0.0017
        self.acc_cov = np.eye(3)*2
        self.acc_bias_co = np.array([0,0,0])
        self.gyr_bias_co = np.array([0,0,0])
        self.acc_ortho_co = np.eye(3)
        self.gyr_ortho_co = np.eye(3)
        self.compensated_acc = np.array([0,0,0])
        self.compensated_gyr = np.array([0,0,0])
        self.gravity = 9.81
    
    def compensate_acc(self, raw_acc):
        self.compensated_acc = self.acc_ortho_co.dot(raw_acc+self.acc_bias_co)
        return self.compensated_acc

    def compensate_gyr(self, raw_gyr):
        self.compensated_gyr = self.gyr_ortho_co.dot(raw_gyr+self.gyr_bias_co)
        return self.compensated_gyr
    
        
    def f(self, x, u, dt):
        '''
        ____________STATE ESTIMATE VECTOR______________

                State                   unit        index
        orientation(quaternion)                     0:3

        ___________COMMANDS VECTOR_____________________

                State                   unit        index
        gyroscopes(xyz)                 rad/s       0:2

        ___________SENSOR VECTORS_____________________

                State                   unit       index
        accelerometers(xyz)             m/s²        3:5

        '''   
        q_ = Quaternion(x[0], x[1], x[2], x[3])
        q_.integrate(u, dt)
        new_x = q_.elements
        return new_x
    
    def h(self, x):
        q_ = Quaternion(x[0], x[1], x[2], x[3])
        return q_.rotate(np.array([0,0,-self.gravity]))
    
    def predict(self, raw_gyr, dt):        
        self.last_prediction = time    
        u = self.compensate_gyr(raw_gyr)
        f = lambda x, u: self.f(x, u, dt)
        self.ekf.predict(f, u, self.gyr_cov)
        self.orientation = Quaternion(self.ekf.x[0], self.ekf.x[1], self.ekf.x[2], self.ekf.x[3])
        
    def update(self, raw_acc):        
        self.last_update = time
        z = self.compensate_acc(raw_acc)
        h = lambda x: self.h(x)
        self.ekf.update(h, z, self.acc_cov)
        self.orientation = Quaternion(self.ekf.x[0], self.ekf.x[1], self.ekf.x[2], self.ekf.x[3])
        
        
class FakeQuad:
    def __init__(self):
        self.m = 0.7
        self.J = np.array([[0.002, 0, 0], [0, 0.002, 0], [0, 0, 0.003]])
        self.engines = [FakeEngine() for _ in range(4)]
        self.engines[0].cw = -1
        self.engines[0].propeller.cw = -1
        self.engines[2].cw = -1
        self.engines[2].propeller.cw = -1
        
        d = 0.23
        self.engines[0].position = np.array([d,-d,0])
        self.engines[1].position = np.array([d,d,0])
        self.engines[2].position = np.array([-d,d,0])   
        self.engines[3].position = np.array([-d,-d,0])
        
        self.state = FakeState()
        self.imu = FakeIMU()
        
        self.rollPid = Pid(0.1, 0.1, 0.005, 0.2, -0.2, 0.1)
        self.pitchPid = Pid(0.1, 0.1, 0.005, 0.2, -0.2, 0.1)
        self.yawPid = Pid(0.20, 0.15, 0.00, 0.2, -0.2, 0.15)
        self.orientation_kp = 10

        self.angular_vel_command = np.array([0,0,0])
        
        
        self.estimator = FakeEstimator()
        self.to_send = {}
        self.last_time_send = 0
        self.send_stream_delay = 50
        
    def forword_pfd(self, u, dt):
        engine_res = np.array([engine.update(u[i], -self.state.omega[2], dt) for i, engine in enumerate(self.engines)])
        #compute the total vector thrust (POA and magnitude)
        mag = np.sum(engine_res[:,0])
        if mag/self.m < 1e-6:
            POA = np.array([0,0,0])
        else:
            POA = (self.engines[0].position*engine_res[0,0]+self.engines[1].position*engine_res[1,0]+self.engines[2].position*engine_res[2,0]+self.engines[3].position*engine_res[3,0])/mag
    
        #compute the total drag torque
        drag_torque = np.sum(engine_res[:,1])
        #compute the total torque
        torque = np.cross(POA, np.array([0,0,mag]))+np.array([0,0,drag_torque])
        #compute the angular acceleration
        self.omega_prime = np.linalg.inv(self.J).dot(torque)
        #compute the angular velocity
        self.state.omega += self.omega_prime*dt
        #integrate the angular velocity to get the orientation
        self.state.orientation.integrate(self.state.omega,dt)

        #compute the linear acceleration
        self.state.acceleration = np.array([0,0,-mag/self.m])
        #rotate the acceleration to the world frame
        # print(acc)
        self.state.acceleration = self.state.orientation.rotate(self.state.acceleration)
        #add gravity
        self.state.acceleration[2] += self.estimator.gravity
        #update the speed
        self.state.speeds += self.state.acceleration*dt
        #update the position
        self.state.position += self.state.speeds*dt
        #reset if the drone is under the ground
        if self.state.position[2] > 0:
            self.state.position *= 0
            self.state.speeds *= 0
            self.state.acceleration *= 0
            self.state.orientation = Quaternion()
            self.state.omega *= 0
            self.state.omega_prime *= 0
            self.estimator.orientation = Quaternion()
            self.estimator.ekf.x = np.array([1,0,0,0])
        
        #compute the acceleration in the quad frame    
        iner_acc = self.state.orientation.inverse.rotate(self.state.acceleration-np.array([0,0,self.estimator.gravity]))
        #simulate the imu
        self.imu.update(iner_acc, self.state.omega, dt)

    def rot_speed_control(self, wx, wy, wz, throttle, dt):

        self.angular_vel_command = np.array([wx, wy, wz])
        crx = self.rollPid.update(wx+self.estimator.compensated_gyr[0], dt)
        cry = self.pitchPid.update(wy+self.estimator.compensated_gyr[1], dt)
        crz = self.yawPid.update(wz+self.estimator.compensated_gyr[2], dt)
        c1 = crx + cry - crz
        c2 = -crx + cry + crz
        c3 = -crx - cry - crz
        c4 = crx - cry + crz
        max_command = max(abs(c1), abs(c2), abs(c3), abs(c4))
        relative_correction = 0.8
        if max_command > 1.0 - relative_correction*throttle or max_command < -relative_correction*throttle:
            scale = (1.0 + relative_correction*throttle)/max(max_command, 1.0 - relative_correction*throttle)
            c1 *= scale
            c2 *= scale
            c3 *= scale
            c4 *= scale
        c1 += throttle
        c2 += throttle
        c3 += throttle
        c4 += throttle
        self.forword_pfd([c1, c2, c3, c4], dt)
    
    def force_angular_vel(self, wx, wy, wz, dt):
        #integrate the angular velocity to get the orientation
        self.state.omega_prime *= 0
        self.state.omega = np.array([wx, wy, wz])
        self.state.orientation.integrate(self.state.omega,dt)
        self.state.acceleration *= 0
        self.state.speeds *= 0
        self.state.position *= 0
        iner_acc = self.state.orientation.inverse.rotate(-np.array([0,0,self.estimator.gravity]))
        self.imu.update(iner_acc, self.state.omega, dt)
    
    def send(self, buffer):
        dt = time.time()-self.last_time_send
        if dt < self.send_stream_delay/1000:
            return
        self.last_time_send = time.time()
        new_data = {}
        new_data["time"] = int(time.time()*1000)
        new_data["gyro_raw"] = self.imu.gyr.tolist()
        new_data["acc_raw"] = self.imu.acc.tolist()
        new_data["orientation"] = self.estimator.orientation.elements.tolist()
        new_data["w1"] = self.engines[0].u
        new_data["w2"] = self.engines[1].u
        new_data["w3"] = self.engines[2].u
        new_data["w4"] = self.engines[3].u
        new_data["angular_velocity_command"] = self.angular_vel_command.tolist()
        buffer.append([new_data])
   
    def receive(self, buffer):
        for data in buffer:
            if data is None:
                continue
            if "orientation" in data:
                self.estimator.orientation = Quaternion(data["orientation"])
            if "gyro_bias_co" in data:
                self.estimator.gyr_bias_co = np.array(data["gyro_bias_co"])
            if "acc_bias_co" in data:
                self.estimator.acc_bias_co = np.array(data["acc_bias_co"])
            if "gyro_scale_co" in data:
                self.estimator.gyr_ortho_co = np.array(data["gyro_scale_co"]).reshape(3,3)
            if "acc_scale_co" in data:
                self.estimator.acc_ortho_co = np.array(data["acc_scale_co"]).reshape(3,3)
            if "send_stream_delay" in data:
                self.send_stream_delay = data["send_stream_delay"]
        buffer = []
            
    def run(self, roll, pitch, rz, throttle, dt):
        
        #update the estimator
        self.estimator.predict(self.imu.gyr, dt)
        self.estimator.update(self.imu.acc)
        
        angle = np.linalg.norm([roll, pitch])
        if abs(angle) < 1e-6:
            axis = np.array([0,0,1])
        else:
            axis = np.array([-roll, -pitch, 0])/angle
        #compute the quaternion from the angle and axis
        q1 = Quaternion(axis=axis, angle=angle)
        #rotate [1,0,0] by the quaternion to get the current forward vector
        forward = self.estimator.orientation.rotate(np.array([1,0,0]))
        yaw = np.arctan2(forward[1], forward[0])
        #compute the current yaw
        q_yaw = Quaternion(axis=np.array([0,0,1]), angle=yaw)
        #compute the wanted orientation
        q = q_yaw*q1
        #interpolate between the current orientation and the wanted orientation
        forward = 0.05
        step = 0.001
        q2 = Quaternion.slerp(self.estimator.orientation, q, amount=forward-step)
        q3 = Quaternion.slerp(self.estimator.orientation, q, amount=forward+step)
        #compute the angular velocity
        qd = (q3-q2)/(2*step)
        w = qd*self.estimator.orientation
        w = -2*np.array([w[1], w[2], w[3]])
        #compute the throttle to compensate the gravity kwowing the orientation
        up = self.estimator.orientation.rotate(np.array([0,0,1]))
        compensated_throttle = throttle/up[2]
        return self.rot_speed_control(self.orientation_kp*w[0], self.orientation_kp*w[1], self.orientation_kp*w[2]+rz, compensated_throttle, dt)