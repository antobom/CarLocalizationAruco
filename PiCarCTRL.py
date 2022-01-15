from picar import front_wheels
from picar import back_wheels
from Position import Position
import picar
import time
import math
from threading import Thread, Lock

class PiCarCTRL(Thread):
    def __init__(self, wheelBase):
        Thread.__init__(self)
        config_file = "parameter_dir/config"
        self.fw = front_wheels.Front_Wheels(db=config_file)
        self.bw = back_wheels.Back_Wheels(db=config_file)

        self.E = wheelBase
        self.pos = Position()
        self.tagetPose = Position()
        self.turn_angle = 0
        self.carCtrlLock = Lock()
    def run(self):
        while True:
            time.sleep(0.25)
            self.trajectoryCtrl()

    def set_target(self, pos, target):
        self.targetPose = target

    def get_turn_angle(self):
        return self.turn_angle

    def trajectoryCtrl(self, pos):
        # [X, Y, alpha] = self.targetPose
        self.pos = pos
        delta_psy = self.pos.yaw - self.targetPose.yaw
        #revoir le calcul de L et H
        L = self.pos.x - self.targetPose.x
        H = self.pos.y - self.targetPose.y

        a = -2*H/L**3 - math.tan(delta_psy)/L**2
        b = 3*H/L**2 - math.tan(delta_psy)/L
        angle = -int( math.atan(3*a*self.E**2 + 2*b*self.E) *180/math.pi )

        if angle>27:
            angle = 27
        elif angle<-27:
            angle = -27
        fw.turn(90+angle)
        self.carCtrlLock.acquire()
        self.turn_angle = angle
        self.carCtrlLock.release()
