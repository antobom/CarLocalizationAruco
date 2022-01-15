import threading
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import xlrd
import time


class Camera():
    def __init__(self):
        self.Picam = PiCamera()
        self.Picam.resolution = (640,480)
        self.Picam.framerate = 32
        self.rawCapture = PiRGBArray(self.Picam, size = self.Picam.resolution)

        self.mtx = np.zeros((3,3))
        self.dist = np.zeros((1,5))
        self.get_parameters()

    # def run(self):
    #     # thread_cam = threading.currentThread()
    #     for frame_cap in self.Picam.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
    #         self.cameraLock.acquire()
    #         self.frame = cv2.flip(frame_cap.array,-1)
    #         self.rawCapture.truncate(0)
    #         self.cameraLock.release()

    def get_parameters(self):
        wb = xlrd.open_workbook("parameter_dir/camera_calibration_matrix.xls")
        sheet1 = wb.sheet_by_name("sheet1")
        i = 0
        compteur = 0

        while i < 3:
            j = 0
            while j < 3:
                self.mtx[i][j] = sheet1.cell(compteur, 0).value
                j += 1
                compteur += 1
            i += 1
        i = 0
        while i < 5:
            self.dist[0][i] = sheet1.cell(compteur, 0).value
            compteur += 1
            i += 1
