#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData
from std_msgs.msg import Header


class PLOT:

    # Static params
    FRAME_ID = "base_link"
    PLOT_TIME = 1

    def __init__(self):
        # OnstartUp
        self.fs = rospy.get_param('~rate', 192000) # 192000

        # Initialize fig
        #self.fig, (self.ax1, self.ax2) = plt.subplots(2,1)
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], color='blue', linestyle='-', linewidth=2)
        #self.line, = self.ax.plot([], [], color='red', marker='o', markersize=10, markeredgecolor='black', linestyle='')
        #self.line1, = self.ax1.plot([], [], linewidth=2)
        #self.line2, = self.ax2.plot([], [], linewidth=2)
        #self.ax.set_label_position()
        #self.ax.axis('auto')
        #self.ax1.set_ylim([-2, 2])
        self.ax.set_xlabel("Time(s)")
        self.ax.set_ylabel("Voltage_test(v)")
        self.ax.set_ylim([0, 10])
        self.ax.set_xlim([0, PLOT.PLOT_TIME])
        #self.ax2.set_xlabel("Time(s)")
        #self.ax2.set_ylabel("Voltage(v)")

        self.t_axis = np.linspace(0, PLOT.PLOT_TIME, self.fs * PLOT.PLOT_TIME)


        # Initialize params
        self.plot_array_ch1 = np.zeros(int(PLOT.PLOT_TIME * self.fs))
        self.plot_array_ch2 = np.zeros(int(PLOT.PLOT_TIME * self.fs))


        # Subscriber
        #rospy.Subscriber("/hydrophone_data", HydrophoneData, self.hydro_cb)

    def hydro_cb(self, msg):

        # Subcribe data
        ch1 = np.array(msg.data_ch1)
        ch2 = np.array(msg.data_ch2)
        print(f'Incoming msg size : {ch1.shape}')
        #print "ch1 shape : ", ch1.shape #(192000,)
        #time_start = rospy.get_time()
        #print "time_start : ", time_start

        # Remove previous data and append new data
        self.plot_array_ch1 = self.plot_array_ch1[ch1.shape[0]:] # (513, 7493)
        self.plot_array_ch2 = self.plot_array_ch2[ch2.shape[0]:] # (513, 7493)
        print(f'After remove : {self.plot_array_ch1.shape}')
        self.plot_array_ch1 = np.concatenate((self.plot_array_ch1,ch1)) # (513, 9375)
        self.plot_array_ch2 = np.concatenate((self.plot_array_ch2,ch2)) # (513, 9375)
        print(f'After append : {self.plot_array_ch1.shape}')


    def animation_frame(self,i):
        #self.line[0].set_data(self.plot_array_ch1,)
        tmp1 = np.ones(self.fs)
        test = np.arange(self.fs)
        tmp2 = tmp1.tolist()
        print(f'x = {i}')
        self.line.set_data(test, self.t_axis)
        print(f'line = {self.line}')
        #self.line[1].set_data(self.plot_array_ch2,)
        #self.line2.set_data(2,2)
        #self.line = [self.line1, self.line2]
        return self.line,
        #self.im.set_array(self.plot_fft_ch1)
        #time_end = int(rospy.get_time() - self.animation_start)
        #time_start = time_end - PLOT.PLOT_FFT_TIME
        #self.im.set_extent([time_start, time_end, 0, self.fs//2//1000])
        ##self.im.set_extent([0,5,0,96])
        #return [self.im]


        
    def onShutdown(self):
        rospy.loginfo("PLOT TIME SERIES node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('plot')
    node = PLOT()
    animation = FuncAnimation(node.fig, node.animation_frame, interval=500, blit=True)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
