#!/usr/bin/env python3
# import python library
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time 

# import ROS library
import rospy
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData

class plot_streaming_data_node:
    def __init__(self):
        # self.time_clear = []
        # self.time_data = []
        # self.time_plot_1 = []
        # self.time_plot_2 = []
        self.getParameters()

        # member variable for subscribing 
        # from /get_hydrophone_data_for2i2/hydrophone_data topic
        self.data_ch1 = np.array([])
        self.data_ch2 = np.array([])
        self.fs = 96000
        self.length = 1024

        # member variable for subscribing  
        # from /compute_fft/fft_data topic
        self.fft_ch1 = []
        self.fft_ch2 = []
        self.delta_f = 0
        self.delta_t = 0

        # plot setting 
        if(self.PLOT_ == 0):
            self.fig, self.ax = plt.subplots(2, 2, figsize=(20,6))
        elif(self.PLOT_ == 1 or self.PLOT_ == 2):
            self.fig, self.ax = plt.subplots(1, 2, figsize=(10,3))
        self.M = 0
        self.N = 0
        self.count_t = 0
        self.count_f = 0

        rospy.Subscriber("/get_sound_data_for2i2/hydrophone_data", HydrophoneData, self.push_hydrophone_data)
        rospy.Subscriber("/compute_fft/fft_data", HydrophoneFFTData, self.push_fft_data)
    # define the function to get the private node parameters setting 
    # for this node from plot_streaming_data.yaml file
    def getParameters(self):
        self.PLOT_ = rospy.get_param("~PLOT_")
        self.PLOT_LENGTH_ = rospy.get_param("~PLOT_LENGTH_")
        self.FPS_ = rospy.get_param("~FPS_")

        self.INTERVAL_ = int(1000/self.FPS_) #(ms)
    def push_hydrophone_data(self, data):
        self.data_ch1 = np.concatenate((self.data_ch1, data.data_ch1))
        self.data_ch2 = np.concatenate((self.data_ch2, data.data_ch2))
        self.fs = data.fs
        self.length = data.length
        number = int(self.fs*self.PLOT_LENGTH_)
        if(len(self.data_ch1)>number):
            self.count_t += len(self.data_ch1)-number
            self.data_ch1 = self.data_ch1[-number:]
            self.data_ch2 = self.data_ch2[-number:]
    def push_fft_data(self, data):
        self.fft_ch1.append(data.fft_ch1)
        self.fft_ch2.append(data.fft_ch2)
        self.delta_t = data.delta_t
        self.delta_f = data.delta_f

        self.M = len(data.fft_ch1)
        self.N = int(self.PLOT_LENGTH_/self.delta_t)
        if(len(self.fft_ch1)>self.N):
            self.count_f += len(self.fft_ch1)-self.N
            self.fft_ch1 = self.fft_ch1[-self.N:]
            self.fft_ch2 = self.fft_ch2[-self.N:]

    def animate_0(self):
        # time_stamp_1 = time.time()
        self.ax[0][0].clear()
        self.ax[0][1].clear()
        self.ax[1][0].clear()
        self.ax[1][1].clear()
        # time_stamp_2 = time.time()

        data_ch1 = self.data_ch1
        data_ch2 = self.data_ch2
        count_t = self.count_t
        fft_ch1 = self.fft_ch1
        fft_ch2 = self.fft_ch2
        count_f = self.count_f

        t1 = (np.arange(len(data_ch1))+count_t)/self.fs
        t2 = (np.arange(len(data_ch2))+count_t)/self.fs
        t3_max = (len(fft_ch1)+count_f)*self.delta_t-self.PLOT_LENGTH_
        t4_max = (len(fft_ch2)+count_f)*self.delta_t-self.PLOT_LENGTH_
        f_max = self.fs/2
        m = [0]*self.M
        range_number = len(fft_ch1)
        if(range_number<self.N):
            for i in range(self.N-range_number):
                fft_ch1.append(m)
                fft_ch2.append(m)
        # time_stamp_3 = time.time()
        self.ax[0][0].plot(t1, data_ch1)
        self.ax[0][0].set_ylabel("Voltage(volt)")
        self.ax[0][0].set_title("Streaming plot for CH1")
        self.ax[0][0].grid(True)

        self.ax[0][1].plot(t2, data_ch2)
        self.ax[0][1].set_ylabel("Voltage(volt)")
        self.ax[0][1].yaxis.set_label_position("right")
        self.ax[0][1].yaxis.set_ticks_position("right")
        self.ax[0][1].set_title("Streaming plot for CH2")
        self.ax[0][1].grid(True)
        # time_stamp_4 = time.time()

        self.ax[1][0].imshow(np.array(fft_ch1).T, origin="lower", extent=[t3_max-self.PLOT_LENGTH_, t3_max, 0, f_max])
        self.ax[1][0].axis('auto')
        self.ax[1][0].set_xlabel("Time(s)")
        self.ax[1][0].set_ylabel("Frequency(Hz)")
        self.ax[1][0].set_ylim([0, 20000])

        self.ax[1][1].imshow(np.array(fft_ch2).T, origin="lower", extent=[t4_max-self.PLOT_LENGTH_, t4_max, 0, f_max])
        self.ax[1][1].axis('auto')
        self.ax[1][1].set_xlabel("Time(s)")
        self.ax[1][1].set_ylabel("Frequency(Hz)")
        self.ax[1][1].set_ylim([0, 20000])
        self.ax[1][1].yaxis.set_label_position("right")
        self.ax[1][1].yaxis.set_ticks_position("right")
        # time_stamp_5 = time.time()
        # self.time_clear.append(time_stamp_2-time_stamp_1)
        # self.time_data.append(time_stamp_3-time_stamp_2)
        # self.time_plot_1.append(time_stamp_4-time_stamp_3)
        # self.time_plot_2.append(time_stamp_5-time_stamp_4)
        # clear = np.mean(np.array(self.time_clear))
        # data = np.mean(np.array(self.time_data))
        # plot1 = np.mean(np.array(self.time_plot_1))
        # plot2 = np.mean(np.array(self.time_plot_2))
        # print("clear consuming time: ", clear)
        # print("data consuming time: ", data)
        # print("plot1 consuming time: ", plot1)
        # print("plot2 consuming time: ", plot2)
        # print("sum consuming time: ", clear+data+plot1+plot2)
    def animate_1(self):
        self.ax[0][0].clear()
        self.ax[0][1].clear()
        data_ch1 = self.data_ch1
        data_ch2 = self.data_ch2
        count_t = self.count_t

        t1 = (np.arange(len(data_ch1))+count_t)/self.fs
        t2 = (np.arange(len(data_ch2))+count_t)/self.fs

        self.ax[0][0].plot(t1, data_ch1)
        self.ax[0][0].set_xlabel("Time(s)")
        self.ax[0][0].set_ylabel("Voltage(volt)")
        self.ax[0][0].set_title("Streaming plot for CH1")
        self.ax[0][0].grid(True)

        self.ax[0][1].plot(t2, data_ch2)
        self.ax[0][1].set_xlabel("Time(s)")
        self.ax[0][1].set_ylabel("Voltage(volt)")
        self.ax[0][1].yaxis.set_label_position("right")
        self.ax[0][1].yaxis.set_ticks_position("right")
        self.ax[0][1].set_title("Streaming plot for CH2")
        self.ax[0][1].grid(True)

    def animate_2(self):
        self.ax[0][0].clear()
        self.ax[0][1].clear()
        fft_ch1 = self.fft_ch1
        fft_ch2 = self.fft_ch2
        count_f = self.count_f

        t3_max = (len(fft_ch1)+count_f)*self.delta_t-self.PLOT_LENGTH_
        t4_max = (len(fft_ch2)+count_f)*self.delta_t-self.PLOT_LENGTH_
        f_max = self.fs/2
        m = [0]*self.M
        range_number = len(fft_ch1)
        if(range_number<self.N):
            for i in range(self.N-range_number):
                fft_ch1.append(m)
                fft_ch2.append(m)

        self.ax[0][0].imshow(np.array(fft_ch1).T, origin="lower", extent=[t3_max-self.PLOT_LENGTH_, t3_max, 0, f_max])
        self.ax[0][0].axis('auto')
        self.ax[0][0].set_xlabel("Time(s)")
        self.ax[0][0].set_ylabel("Frequency(Hz)")
        self.ax[0][0].set_ylim([0, 20000])

        self.ax[0][1].imshow(np.array(fft_ch2).T, origin="lower", extent=[t4_max-self.PLOT_LENGTH_, t4_max, 0, f_max])
        self.ax[0][1].axis('auto')
        self.ax[0][1].set_xlabel("Time(s)")
        self.ax[0][1].set_ylabel("Frequency(Hz)")
        self.ax[0][1].set_ylim([0, 20000])
        self.ax[0][1].yaxis.set_label_position("right")
        self.ax[0][1].yaxis.set_ticks_position("right")

    def animate(self, i, case):
        time_start = time.time()
        if(case == 0):
            self.animate_0()
        elif(case == 1):
            self.animate_1()
        elif(case == 2):
            self.animate_2()
        time_end = time.time()
        print("total consuming: ", time_end-time_start, "sec")



    

def main():
    rospy.init_node("plot_streaming_data_node", anonymous=True)
    plot_node = plot_streaming_data_node()
    if(plot_node.PLOT_ == 0):
        # plot both 
        ani = FuncAnimation(plot_node.fig, plot_node.animate, fargs=(0,), interval=plot_node.INTERVAL_)
    elif(plot_node.PLOT_ == 1):
        # only plot time series
        ani = FuncAnimation(plot_node.fig, plot_node.animate, fargs=(1,), interval=plot_node.INTERVAL_)
    elif(plot_node.PLOT_ == 2):
        # only plot stft 
        ani = FuncAnimation(plot_node.fig, plot_node.animate, fargs=(2,), interval=plot.node.INTERVAL_)
    plt.show()
    rospy.spin()

if __name__ == "__main__":
    main()
