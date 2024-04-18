import numpy as np


"""
predict: 
x = f(x,u)
P = Fx*P*Fx^T + Fu*Q*Fu^T

update:
y = z - h(x)
S = H*P*H^T + R
K = P*H^T*S^-1
x = x + K*y
P = (I - K*H)*P
"""
class Ekf:
    
    def jacobian(self, vector_function, x, y0,  h=1e-5):
        n = len(x)
        m = len(y0)
        J = np.zeros((m,n))
        for i in range(n):
            dx = np.zeros(n)
            dx[i] = h
            y1 = vector_function(x + dx)
            J[:,i] = (y1 - y0).flatten()/h
        return J
    
    
    def __init__(self,x_dim=0):
        self.x_dim = x_dim    
        self.x = np.zeros(x_dim)
        self.P = np.eye(x_dim)
        
    def predict(self,f, u, Q):
        prev_x = self.x
        self.x = f(prev_x, u)
        Fx = self.jacobian(lambda x: f(x, u), prev_x, self.x)
        Fu = self.jacobian(lambda uu: f(prev_x, uu), u, self.x)
        self.P = Fx @ self.P @ Fx.T + Fu @ Q @ Fu.T
        
    def update(self, h, z, R):
        h_pred = h(self.x)
        H = self.jacobian(h, self.x, h_pred)
        y = z - h_pred
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(self.x_dim) - K @ H) @ self.P
        #return the measurement prediction and its covariance
        return h_pred, S
        
    def mahalanobis(self, h, z, R):
        h_pred = h(self.x)
        y = z - h_pred
        H = self.jacobian(h, self.x, h_pred)
        S = H @ self.P @ H.T + R
        return y.T @ np.linalg.inv(S) @ y
    
    def __str__(self) -> str:
        return f"x: {self.x}\nP: {self.P}"

if __name__ == "__main__":
    
    import matplotlib.pyplot as plt
    
    dt=0.1
    
    def f(X,U):
        x = X[0]
        y = X[1]
        theta = X[2]
        v = U[0]
        w = U[1]
        x = x + v*np.cos(theta) * dt
        y = y + v*np.sin(theta) * dt
        theta = theta + w*dt
        return np.array([x,y,theta])
    
    def h(X):
        return X[:2]
    
    def h2(X):
        x = X[0]
        y = X[1]
        return np.array([np.sqrt(x**2 + y**2)])
    
    T=20
    
    t = np.arange(0,T,dt)
    
    #generate trajectory from polar coordinates
    r= np.linspace(1,0.6,len(t))**2
    theta = np.linspace(0,4*np.pi,len(t))
    
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    
    direction = np.zeros((2,len(t)))
    direction[0,1:] = (x[1:] - x[:-1])/dt
    direction[1,1:] = (y[1:] - y[:-1])/dt
    direction[0,0] = direction[0,1]
    direction[1,0] = direction[1,1]
    theta = np.arctan2(direction[1,:],direction[0,:])
    
    #compute control inputs
    omega = np.zeros(len(t))
    omega[1:] = (theta[1:] - theta[:-1])/dt
    omega[0] = omega[1]
    velocity = np.sqrt(direction[0,:]**2 + direction[1,:]**2)
    
    X=np.array([x,y,theta])
    
    #simulate noisy measurements
    gpsNoise = 0.1
    z1 = h(X) + np.random.normal(0,gpsNoise,(2,len(t)))
    R1 = gpsNoise**2*np.eye(2)
    
    beaconNoise = 0.05
    z2 = h2(X) + np.random.normal(0,beaconNoise,(1,len(t)))
    R2 = beaconNoise**2*np.eye(1)
    
    #simulate noisy control inputs
    velocityNoise = 0.01
    omegaNoise = 0.01
    U = np.array([velocity+np.random.normal(0,velocityNoise,len(t)),omega+np.random.normal(0,omegaNoise,len(t))])
    Q = np.diag([velocityNoise**2,omegaNoise**2])
    
    
    EkfX = np.zeros((3,len(t)))
    EkfP = np.zeros((3,3,len(t)))

    ekf = Ekf(3)
    ekf.x = np.array([z1[0,0],z1[1,0],np.pi/2])
    ekf.P = np.zeros((3,3))
    ekf.P[0,0] = 0.1
    ekf.P[1,1] = 0.1
    ekf.P[2,2] = 0.1
    
    EkfX[:,0] = ekf.x
    EkfP[:,:,0] = ekf.P
    
    for i in range(1,len(t)):
        ekf.predict(f, U[:,i], Q)
        ekf.update(h, z1[:,i], R1)
        ekf.update(h2, z2[:,i], R2)
        EkfX[:,i] = ekf.x
        EkfP[:,:,i] = ekf.P
        
    plt.figure()
    plt.plot(x,y)
    plt.plot(EkfX[0,:],EkfX[1,:])
    plt.scatter(z1[0,:],z1[1,:])
    plt.show()
    

    
    
   