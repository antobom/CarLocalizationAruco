import numpy as np
import xlrd
import time


class Accelerometer:
    def __init__(self):
        self.time_list = []
        self.long_acc_list = []
        self.lat_acc_list = []
        self.yaw_speed_list = []

        self.time = 0
        self.lat_acc = 0
        self.long_acc = 0
        self.yaw_speed = 0
        self.dt = time.time()
        self.iterator = 0
        self.read_generated_data()

    def read_generated_data(self):
        wb = xlrd.open_workbook("parameter_dir/generatedData.xlsx")
        sheet = wb.sheet_by_name("generatedData")

        time = sheet.col(0, 1)
        long_acc_list = sheet.col(5, 1)
        lat_acc_list = sheet.col(6, 1)
        yaw_speed_list = sheet.col(7, 1)

        for i in range(len(time)):
            self.time_list.append(time[i].value)
            self.long_acc_list.append(long_acc_list[i].value)
            self.lat_acc_list.append(lat_acc_list[i].value)
            self.yaw_speed_list.append(yaw_speed_list[i].value)

    def get_current_state(self):
        i = self.iterator
        self.iterator +=1

        self.time += time.time() - self.dt
        self.dt = time.time()

        if len(self.time_list)>i:#in order to stay in array range
            while self.time<self.time_list[i]:#wait until the next value is reached
                time.sleep(0.005)
                self.time+= time.time()-self.dt
                self.dt = time.time()
            return np.array([self.long_acc_list[i], self.lat_acc_list[i], self.yaw_speed_list[i]])

        else:
            self.iterator = 0
            self.time = 0
            print("acceleration data reset")
            for i in range(len(self.long_acc_list)):
                self.long_acc_list[i] = 0
            return np.array([self.long_acc_list[0], self.lat_acc_list[0], self.yaw_speed_list[0]])

#
# start_time = time.time()
#
#
# value = np.array([1, 0 ])
# data_long, data_lat = [], []
# time_plot = []
# i=0
# dt = 0
# duration = time.time()
# acc = Accelerometre()
#
# while value is not None:
#
#     value = acc.get_current_state()
#     if value is not None :
#         time_plot.append(value[0])
#         data_long.append(value[1])
#         data_lat.append(value[2])
# print(time.time() - duration, len(data_long))
