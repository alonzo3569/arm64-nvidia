#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData, Tdoa
from std_msgs.msg import Header


class PLOT:

    # Static params
    FRAME_ID = "base_link"
    PLOT_TIME = 5

    def __init__(self):
        # OnstartUp
        self.fs = rospy.get_param('~rate', 192000) # 192000
        self.tdoa_window_length= rospy.get_param('~tdoa_window_length', 1.0) #sec

        # Initialize params
        self.plot_array_ch1 = np.array([])
        self.plot_array_ch2 = np.array([])
        self.t_axis = np.linspace(0, self.tdoa_window_length, int(self.fs * self.tdoa_window_length))
        self.threshold = 0.0
        self.ch1_avg = 0.0
        self.threshold_line = np.array([])   

        # Initialize fig
        self.fig, self.ax1 = plt.subplots()
        self.line1, = self.ax1.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line2, = self.ax1.plot([], [], color='red', linestyle='-', linewidth=1)
        self.line = [self.line1, self.line2]
        #self.line, = self.ax.plot([], [], color='red', marker='o', markersize=10, markeredgecolor='black', linestyle='')
        #self.ax.set_label_position()
        #self.ax.axis('auto')

        self.ax1.set_ylim([-1, 1])
        self.ax1.set_xlim([0, self.tdoa_window_length])
        self.ax1.set_xlabel("Time(s)")
        self.ax1.set_ylabel("Voltage(volts)")

        # Subscriber
        rospy.Subscriber("tdoa", Tdoa, self.tdoa_cb)

    def tdoa_cb(self, msg):

        # Subcribe data
        ch1 = np.array(msg.data_ch1)
        ch2 = np.array(msg.data_ch2)
        self.threshold = msg.threshold
        self.ch1_avg = msg.ch1_avg
        print(f'ch1_avg : {self.ch1_avg}')
        #print(f'Incoming msg size : {ch1.shape}')
        #print "ch1 shape : ", ch1.shape #(192000,)
        #time_start = rospy.get_time()
        #print "time_start : ", time_start

        # Remove previous data and append new data
        self.plot_array_ch1 = ch1
        self.plot_array_ch2 = ch2

        # Calculate threshold
        self.threshold_line = self.threshold * self.ch1_avg * np.ones(ch1.shape[0])
        #print(f'After remove : {self.plot_array_ch1.shape}')
        #print(f'After append : {self.plot_array_ch1.shape}')


    def animation_frame(self,i):

        print(f'shape of t axis          {self.t_axis.shape}')
        print(f'shape of plot array      {self.plot_array_ch1.shape}')
        print(f'shape of threshold line  {self.threshold_line.shape}')
        # Notice : t_axis & ch1 should be same size

        if self.t_axis.shape[0] == self.plot_array_ch1.shape[0]:
            self.line[0].set_data(self.t_axis, self.plot_array_ch1)
            self.line[1].set_data(self.t_axis, self.threshold_line)

        return self.line


        
    def onShutdown(self):
        rospy.loginfo("PLOT TIME SERIES node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('plot')
    node = PLOT()
    animation = FuncAnimation(node.fig, node.animation_frame, interval=500, blit=True)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
