import socket
from threading import Thread, Lock
import pickle
import numpy as np
from time import sleep



class ClientSocket(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.hote = '192.168.43.167'
        self.port = 12800
        self.client = socket.socket()
        self.client.connect((self.hote,self.port))
        self.clientLock = Lock()
        self.msg = ''
        print("client connected")
        self.new_pos = np.zeros((4,1))
        self.new_cov = np.zeros((4,4))

    def run(self):
        while True:
            sleep(0.05)
            self.send_data()

    def send_data(self):
        self.clientLock.acquire()
        msg_pos = pickle.dumps(self.new_pos)
        msg_cov = pickle.dumps(self.new_cov)
        self.clientLock.release()

        reponse = ''
        self.client.send(msg_pos)
        while reponse != "ok_pos":
            reponse = self.client.recv(1024).decode()

        self.client.send(msg_cov)
        while reponse != "ok_cov":
            reponse = self.client.recv(1024).decode()

    def set_position(self, data):
        self.new_pos = data
    def set_cov(self, cov):
        self.new_cov = cov
