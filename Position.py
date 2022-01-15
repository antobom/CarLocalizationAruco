import numpy as np
class Position:
    def __init__(self, x=0, y=0, v=0, yaw=0):
        self.x = x
        self.y = y
        self.v = v
        self.yaw = yaw
        self.position_array = np.array([self.x, self.y, self.v, self.yaw])
        
    def get_position_array(self):
        self.position_array = np.array([self.x, self.y, self.v, self.yaw])
        return self.position_array

    def print_position(self):
        print("x:", self.x, " y:", self.y, " v:", self.v, " yaw:", self.yaw)
