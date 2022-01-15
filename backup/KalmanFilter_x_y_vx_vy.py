import numpy as np

class KalmanFilter:
    def __init__(self, dt=1.0):
        self.A = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])
        self.B = np.array([[ (dt**2)/2, 0], [dt, 0], [0, (dt**2)/2], [0, dt]])
        # self.C = np.array([[1,0],[0,0]])
        self.C = np.array([ [1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0] ])
        # self.Pk_1 = np.array([[400,0],[0,25]])
        self.I = np.eye(4)
        self.H = self.I
        self.R = np.array([[0.083672002407409, 0, 0.004553885853386, 0],
                           [0, 1, 0, 0],
                           [0.004553885853386, 0, 0.083672002407409, 0],
                           [0, 0, 0, 1]])#0.002172223815664
        self.Pk = np.array([[3.30297e-08,0,0,0],
                          [0,2.31558e-05,0,0],
                          [0,0,1,0],
                          [0,0,0,1]])
        self.Xk = np.array([
                        [0.4],
                        [0],
                        [-0.2],
                        [0]
                            ])
    def prediction(self, U):
        self.Xk = self.A @ self.Xk + self.B @ U
        self.Pk = self.A @ self.Pk @ self.A.T

    def correction(self, Ykm):
        Yk = self.C @ Ykm
        #TO DO check if matrix is invertible
        K = self.Pk @ self.H @ np.linalg.inv((self.H @ self.Pk @ self.H.T) + self.R)
        self.Xk = self.Xk + K @ (Yk - self.H @ self.Xk)
        self.Pk = (self.I - K @ self.H) @ self.Pk

    def getState(self):
        return [self.Xk[0,0],self.Xk[1,0],self.Xk[2,0],self.Xk[3,0]]
