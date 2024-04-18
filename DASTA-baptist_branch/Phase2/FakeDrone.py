import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
import time

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
        
class FakeQuad:
    def __init__(self):
        self.m = 0.7
        self.J = np.array([[0.002, 0, 0], [0, 0.002, 0], [0, 0, 0.003]])
        self.engines = [FakeEngine() for i in range(4)]
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
        self.angular_vel_command = np.array([0,0,0])
        
    def rotSpeedControl(self, wx, wy, wz, throttle, dt):
        '''
        float Crx = pidRx.compute(communication.angular_velocity_command.data[0] - sensors.gyro.data[0], time);
        float Cry = pidRy.compute(communication.angular_velocity_command.data[1] - sensors.gyro.data[1], time);
        float Crz = pidRz.compute(communication.angular_velocity_command.data[2] - sensors.gyro.data[2], time);

        float c1 = Crx + Cry - Crz;
        float c2 = -Crx + Cry + Crz;
        float c3 = -Crx - Cry - Crz;
        float c4 = Crx - Cry + Crz;

        // Find the maximum absolute value among the commands
        float max_command = max(max(max(abs(c1), abs(c2)), abs(c3)), abs(c4));

        float relative_correction = 0.8; // 80% of the thrust maximum allowed for correction
        // Scale the commands to ensure they stay within the range of [-r*thrust, 1.0-r*thrust]
        if (max_command > 1.0 - relative_correction*thrust || max_command < -relative_correction*thrust) {
            float scale = (1.0 + relative_correction*thrust) / max(max_command, 1.0 - relative_correction*thrust);
            c1 *= scale;
            c2 *= scale;
            c3 *= scale;
            c4 *= scale;
        }
    '''
        
        self.angular_vel_command = np.array([wx, wy, wz])
        
        Crx = self.rollPid.update(wx+self.state.omega[0], dt)
        Cry = self.pitchPid.update(wy+self.state.omega[1], dt)
        Crz = self.yawPid.update(wz+self.state.omega[2], dt)
        # Crx = wx
        # Cry = wy
        # Crz = wz
        
        c1 = Crx + Cry - Crz
        c2 = -Crx + Cry + Crz
        c3 = -Crx - Cry - Crz
        c4 = Crx - Cry + Crz
        
        up = self.state.orientation.rotate(np.array([0,0,1]))
        compensated_throttle = throttle/up[2]
        
        max_command = max(abs(c1), abs(c2), abs(c3), abs(c4))
        relative_correction = 0.8
        if max_command > 1.0 - relative_correction*compensated_throttle or max_command < -relative_correction*compensated_throttle:
            scale = (1.0 + relative_correction*compensated_throttle)/max(max_command, 1.0 - relative_correction*compensated_throttle)
            c1 *= scale
            c2 *= scale
            c3 *= scale
            c4 *= scale
            
        c1 += compensated_throttle
        c2 += compensated_throttle
        c3 += compensated_throttle
        c4 += compensated_throttle
        
        
        return self.update([c1, c2, c3, c4], dt)
        
    def run(self, roll, pitch, rz, throttle, dt):
        angle = np.linalg.norm([roll, pitch])
        if abs(angle) < 1e-6:
            axis = np.array([0,0,1])
        else:
            axis = np.array([-roll, -pitch, 0])/angle
        #compute the quaternion from the angle and axis
        q1 = Quaternion(axis=axis, angle=angle)
        #rotate [1,0,0] by the quaternion to get the current forward vector
        forward = self.state.orientation.rotate(np.array([1,0,0]))
        yaw = np.arctan2(forward[1], forward[0])
        q_yaw = Quaternion(axis=np.array([0,0,1]), angle=yaw)
        q = q_yaw*q1
        #compute the current yaw
        forward = 0.05
        step = 0.001
        q2 = Quaternion.slerp(self.state.orientation, q, amount=forward-step)
        q3 = Quaternion.slerp(self.state.orientation, q, amount=forward+step)
        #compute the angular velocity
        qd = (q3-q2)/(2*step)
        w = qd*self.state.orientation
        w = -2*np.array([w[1], w[2], w[3]])
        # print(w)
        return self.rotSpeedControl(10*w[0], 10*w[1], rz, throttle, dt)
    
    def update(self, u, time_step, gravity=9.81):
        EngineRes = np.array([engine.update(u[i], -self.state.omega[2], time_step) for i, engine in enumerate(self.engines)])
        #compute the total vector thrust (POA and magnitude)
        mag = np.sum(EngineRes[:,0])
        if mag/self.m < 1e-6:
            POA = np.array([0,0,0])
        else:
            POA = (self.engines[0].position*EngineRes[0,0]+self.engines[1].position*EngineRes[1,0]+self.engines[2].position*EngineRes[2,0]+self.engines[3].position*EngineRes[3,0])/mag
    
        #compute the total drag torque
        drag_torque = np.sum(EngineRes[:,1])
        #compute the total torque
        torque = np.cross(POA, np.array([0,0,mag]))+np.array([0,0,drag_torque])
        #compute the angular acceleration
        alpha = np.linalg.inv(self.J).dot(torque)
        #compute the angular velocity
        self.state.omega += alpha*time_step
        #integrate the angular velocity to get the orientation
        self.state.orientation.integrate(self.state.omega,time_step)

        #compute the linear acceleration
        acc = np.array([0,0,-mag/self.m])
        #rotate the acceleration to the world frame
        # print(acc)
        acc = self.state.orientation.rotate(acc)
        #add gravity
        acc[2] += gravity
        #update the speed
        self.state.speeds += acc*time_step
        #update the position
        self.state.position += self.state.speeds*time_step
        if self.state.position[2] > 0:
            self.state.position *= 0
            self.state.speeds *= 0
            self.state.omega *= 0
            acc = np.array([0,0,0])
            self.state.orientation = Quaternion()
        inerAcc = self.state.orientation.inverse.rotate(acc-np.array([0,0,gravity]))
        self.imu.update(inerAcc, self.state.omega, time_step)
        return self.state