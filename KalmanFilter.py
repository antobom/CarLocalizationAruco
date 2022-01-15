import numpy as np
import time
import math

class KalmanFilter:

    def __init__(self, wheelBase, Xk = np.zeros((4,1))):

        self.wheelBase = wheelBase

        self.Xk = Xk
        self.Pk = np.eye(4)


        self.C = np.array([ [1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 1] ])

        self.H = np.array([ [1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 1] ])
        self.I = np.eye(4)
        self.Q = np.eye(4)

        self.Q = np.array([[0.001, 0, 0, 0],
                           [0, 0.001, 0, 0],
                           [0, 0, 0.01, 0],
                           [0, 0, 0, 1]])#0.002172223815664
        self.R = np.eye(3)


        self.start_time = time.time()

    def prediction(self, U, turn_angle):
        self.dt = time.time() - self.start_time
        self.start_time = time.time()
        self.Q = np.array([ [self.dt * 4.22829e-06, 0, 0, 0],
                            [0, self.dt * 4.22829e-06, 0, 0],
                            [0, 0, self.dt * 4.22829e-06, 0],
                            [0, 0, 0, 1] ])
        v = self.Xk[2,0]
        yaw = self.Xk[3,0]
        # yaw_point = 2*math.pi/10

        vit = np.array([[-self.dt * v * math.cos(yaw)],
                        [ self.dt * v * math.sin(yaw)],
                        [0],
                        [v * math.tan(turn_angle) / self.wheelBase]])#math.atan(U[0]/U[1])

        acc = np.array([[-self.dt**2/2 * U[0] * math.cos(yaw)],
                        [ self.dt**2/2 * U[1] * math.sin(yaw)],
                        [ self.dt * U[0] ],
                        [- self.dt * U[2]] ])

        self.Xk = self.Xk + vit + acc
        self.Xk[3,0] = self.Xk[3,0]%(2*math.pi)
        jaco = self.compute_jacobian(self.Xk)
        # print(jaco)
        self.Pk = jaco @ self.Pk @ jaco.transpose() + self.Q

    def correction(self, Ykm, R=np.eye((3))):
        self.Yk = self.C @ Ykm #3x1
        # H = self.compute_jacobian(self.Yk)
        self.R = R #3x3
        print(R)
        # H = np.eye(4) #3x4
        Sk = self.H @ self.Pk @ self.H.transpose() + self.R #3x3
        K = self.Pk @ self.H.transpose() @ np.linalg.inv(Sk) #4x3

        self.Xk = self.Xk + K @ ( self.Yk - self.H @self.Xk)
        self.Pk = (self.I - K @ self.H) @ self.Pk

    def getState(self):
        return self.Xk
    def getCov(self):
        return self.Pk

    def compute_jacobian(self, Xk):
        v = Xk[2,0]
        yaw = Xk[3,0]

        jaco = np.array([
                        [1.0, 0, -self.dt * math.cos(yaw),  self.dt * v * math.sin(yaw)],
                        [0, 1.0,  self.dt * math.sin(yaw),  self.dt * v * math.cos(yaw)],
                        [0, 0, 1.0,  0],
                        [0, 0, 0, 1.0 ]  ])

        return jaco
