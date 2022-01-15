import threading
from threading import Thread, Lock

from Camera import Camera
from ArUco import ArUco, ArucoMarker
from KalmanFilter import KalmanFilter
from ClientSocket import ClientSocket
from PiCarCTRL import PiCarCTRL
from Position import Position
from Accelerometer import Accelerometer

import cv2
import time
import numpy as np
import math


class Car(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.wheelBase = 0.173

        self.clientVerbose = True
        self.carCTRLVerbose = False
        self.accelrationVerbose = True

        self.position = Position(1.5, 0.4, 0, math.pi)
        self.cpt = 0

        self.position.print_position()

        self.aruco = ArUco(self.wheelBase)
        self.kalman = KalmanFilter(self.wheelBase, self.position.get_position_array().reshape(4,1))

        if self.clientVerbose:
            self.client = ClientSocket()
            self.client.start()
        if self.carCTRLVerbose:
            self.carCtrl = PiCarCTRL()
            self.carCTRL.start()
            self.carCTRL.set_target(Position(0.5,0.5))


        self.aruco.start()
        time.sleep(3)#wait for al threads to start

        if self.accelrationVerbose:
            self.acc = Accelerometer()

    def run(self):
        print("running...")
        while True:
            car_position, yaw, R = self.aruco.get_data_and_set_state(np.array([self.position.x, self.position.y, self.position.v, self.position.yaw]))
            # prediction phase, the IMU is needed here
            if self.accelrationVerbose:
                U = self.acc.get_current_state()
            else:
                U = np.array([0,0,0])

            if self.carCTRLVerbose:
                turn_angle = self.carCTRL.get_turn_angle()
            else:
                turn_angle = 0

            self.kalman.prediction(U, turn_angle)
            if car_position is not None:
                self.position.x, self.position.y, self.position.yaw = car_position[0, 0], car_position[1, 0], yaw
                Ykm = np.array([
                            [self.position.x],
                            [self.position.y],
                            [0],
                            [self.position.yaw] ])

                self.kalman.correction(Ykm, R)
            state = self.kalman.getState()

            self.position.x, self.position.y, self.position.v, self.position.yaw = state[0,0], state[1,0], state[2,0], state[3,0]
            if not self.clientVerbose:
                self.position.print_position()


            if self.clientVerbose:
                self.client.set_position(self.position.get_position_array())#set the data to send to the computer
                self.client.set_cov(self.kalman.getCov())
            if self.carCTRLVerbose:
                self.carCTRL.trajectoryCtrl(position)

car = Car()
car.start()

    # while True:
#     frame = car.camera.get_frame()
#     cv2.imshow("frame",frame)
#     if cv2.waitKey(1) == ord('q'):
#         cv2.destroyAllWindows()
#         break
