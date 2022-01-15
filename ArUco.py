import cv2
from cv2 import aruco
import numpy as np
import threading
from threading import Thread,Lock
from Camera import Camera
import time
import xlrd
import math


class ArucoMarker():
    def __init__(self, ids, tranlat, rota, psy):
        self.ids = ids
        self.tranlat = tranlat
        self.rota = rota
        self.psy = psy
class ArUco(Thread):
    def __init__(self, wheelBase):
        Thread.__init__(self)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        self.marker_size = 0.1
        self.markerList = []
        self.camera = Camera()
        self.readMarkerList()

        self.arucoLock = Lock()
        self.newData = False
        self.wheelBase = wheelBase
        time.sleep(3)
        self.state = np.array([0, 0, 0, 0])
        self.ambiguity_counter = 0
        self.readVarianceCoef()
    def run(self):
        thread_aruco = threading.currentThread()

        for frame_cap in self.camera.Picam.capture_continuous(self.camera.rawCapture, format="bgr", use_video_port=True):

            frame = cv2.flip(frame_cap.array, -1)
            self.camera.rawCapture.truncate(0)

            rvecs, tvecs, ids = self.detectArucos(frame)
            if len(ids)!=0:
                R_kalman = np.zeros((3,3))
                tCar = np.zeros((3,3))
                psyCar = 0
                for i in range(len(ids)):
                    psy, t, R = self.compute_position(ids[i], rvecs[i], tvecs[i])
                    psyCar += psy
                    tCar += t
                    R_kalman += R
                self.arucoLock.acquire()
                self.newData = True
                self.position = tCar / len(ids)
                # self.rotation = rCar / len(ids)
                self.R_kalman = R_kalman/ len(ids)
                self.yaw_angle = psyCar/len(ids)
                self.arucoLock.release()

                # print(self.position.T)
                # self.newData = False

    def detectArucos(self,frame):
        if frame is not None:
            corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters = self.parameters)
            if len(corners)!=0:
                rvecs,tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera.mtx, self.camera.dist)
                ids = ids[:, 0]
                return rvecs, tvecs, ids
        return [], [], []

    def readVarianceCoef(self):
        wb = xlrd.open_workbook("parameter_dir/coef_variance.xlsx")
        VxSheet = wb.sheet_by_name("Vx")
        VySheet = wb.sheet_by_name("Vy")
        Vx_coef = VxSheet.col(0)
        Vy_coef = VySheet.col(0)
        self.Vx_coef = np.zeros(len(Vx_coef))
        self.Vy_coef = np.zeros(len(Vx_coef))

        for i in range(len(self.Vx_coef)):
            self.Vx_coef[i] = Vx_coef[i].value
            self.Vy_coef[i] = Vy_coef[i].value

    def compute_position(self, ids, rvecs, tvecs):
        ArucoTranlation, ArucoRotation, psy_aruco = self.get_aruco_from_id(ids)
        rvecs, tvecs = rvecs[0], tvecs[0]
        extrinsics = np.matrix(np.zeros((4,4)))
        tvecs[2] += self.wheelBase

        extrinsics[:3,:3] = cv2.Rodrigues(rvecs)[0]
        extrinsics[3,3] = 1
        extrinsics[:3,3] = tvecs.reshape(3,1)
        # t = ArucoTranlation - Rmtx @ tvecs
        extrinsics_inv = extrinsics.I
        T_aruco_ref = np.array([
                    [extrinsics_inv[2, 3]],#x
                    [extrinsics_inv[0, 3]],#y
                    [extrinsics_inv[1, 3]]#z
                    ])
        # print(ArucoRotation)
        R = extrinsics_inv[:3,:3] #rearange the Rotation matrix
        psy = -(self.rot_mat_to_euler(R) + psy_aruco)
        # T_abs = ArucoRotation @ T_aruco_ref + ArucoTranlation#T_aruco_ref is the position estimated by the aruco

        T_aruco_ref = self.remove_ambiguity(T_aruco_ref, ArucoRotation, ArucoTranlation)

        T_abs = ArucoRotation @ T_aruco_ref + ArucoTranlation
        #--------
        # confidence = 1/math.sqrt(extrinsics[0,3]**2 + extrinsics[1,3]**2+extrinsics[2,3]**2)
        x,y = T_aruco_ref[0,0], T_aruco_ref[1,0]
        XY = np.array([x**2 * y**2, x * y**2, y**2, x**2 * y, x * y, y, x**2, x, 1]).reshape(9,1)
        Vx = self.Vx_coef @ XY
        Vy = self.Vy_coef @ XY

        R_kalman = np.array([
                            [Vx[0], 0, 0],
                            [0, Vy[0], 0],
                            [0, 0,  1] ])
        # print(T.reshape(1,3), yaw_angle)
        # print(Vx, Vy,"\n", R_kalman)
        # if T_abs[2,0]>0 or T_abs[1,0]<0:
        #     print(T_abs.transpose())
        return psy, T_abs, R_kalman

    def remove_ambiguity(self, T_aruco_ref, ArucoRotation, ArucoTranlation):
        # get the real position (from the state) in the aruco frame to compare it with the mesure :
        if T_aruco_ref[2,0]>0:
            T_aruco_ref[2,0]=-T_aruco_ref[2,0]
            T_aruco_ref[1,0] = -T_aruco_ref[1,0]

        # if T_aruco_ref[0]>1.0 and abs(T_aruco_ref[1])>0.2:
        #     self.arucoLock.acquire()
        #     T_real_abs = np.array( [self.state[0], self.state[1], 0]) #T_real_abs is the position of the car
        #     self.arucoLock.release()
        #     T_real_aruco_ref = (np.linalg.inv(ArucoRotation) @ (T_real_abs - ArucoTranlation))[0,:]
        #     #T_real_aruco_ref the position of the car in the aruco frame of reference
        #     # print(T_real_aruco_ref, T_aruco_ref[1])
        #     difference = abs(T_real_aruco_ref[1] - T_aruco_ref[1])#we compute the differences with and without correction
        #     difference_corr = abs(T_real_aruco_ref[1] + T_aruco_ref[1])#to compare them and take the tiniest difference
        #     # print(T_real_aruco_ref)
        #     if difference_corr<difference:
        #         T_aruco_ref[1] = -T_aruco_ref[1]
        #         print("ambiguity corrected\n")



        return T_aruco_ref

    def readMarkerList(self):
        wb = xlrd.open_workbook("position_aruco/position_aruco_castelnau.xlsx")
        sheet = wb.sheet_by_name("Aruco_position")
        ids = sheet.col(0,1)
        Ax0 = sheet.col(1,1)
        Ay0 = sheet.col(2,1)
        psy = sheet.col(3,1)
        for i in range(len(ids)):
            ids[i] = ids[i].value
            tranlat = np.array([ [Ax0[i].value], [Ay0[i].value], [0] ])
            rota = np.array([[np.cos(psy[i].value), -np.sin(psy[i].value), 0],
                             [np.sin(psy[i].value),  np.cos(psy[i].value), 0],
                             [0,                     0,                    1] ])

            self.markerList.append(ArucoMarker(ids[i],tranlat, rota, psy[i].value))

    def get_aruco_from_id(self,id):
        for i in range(len(self.markerList)):
            if id == self.markerList[i].ids:
                return self.markerList[i].tranlat, self.markerList[i].rota, self.markerList[i].psy
        print("aruco found but not known, please add the aruco in the specification file")
        assert(False)

    def rot_mat_to_euler(self,rot_mat):
        sy = math.sqrt(rot_mat[0, 0] * rot_mat[0, 0] + rot_mat[1, 0] * rot_mat[1, 0])
        if not sy < 1e-6:
            # teta = math.atan2(rot_mat[2, 1], rot_mat[2, 2])
            psy = math.atan2(-rot_mat[2, 0], sy)
            # phi = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
        else:
            # teta = math.atan2(-rot_mat[1, 2], rot_mat[1, 1])
            psy = math.atan2(-rot_mat[2, 0], sy)
            # phi = 0
        return psy

    def get_data_and_set_state(self, state):
        position = None
        yaw = None
        R = None
        self.arucoLock.acquire()
        self.state = state
        if self.newData:
            position = self.position
            yaw = self.yaw_angle
            R = self.R_kalman
            self.newData = False
        self.arucoLock.release()

        return position, yaw, R
