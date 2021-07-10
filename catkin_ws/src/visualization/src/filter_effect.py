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
        self.plot_array_ch1_filtered = np.array([])
        self.plot_array_ch1_raw = np.array([])
        self.t_axis = np.linspace(0, self.tdoa_window_length, int(self.fs * self.tdoa_window_length))
        self.threshold = 0.0
        self.ch1_avg = 0.0
        self.threshold_line_ch1 = np.array([])   

        # Initialize fig
        self.fig, (self.ax1,self.ax2, self.ax3) = plt.subplots(3,1)
        self.line1, = self.ax1.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line2, = self.ax2.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line3, = self.ax3.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line4, = self.ax3.plot([], [], color='red', linestyle='-', linewidth=1)
        self.line = [self.line1, self.line2, self.line3, self.line4]

        self.ax1.set_ylim([-1, 1])
        self.ax1.set_xlim([0, self.tdoa_window_length])
        self.ax1.set_xlabel("Time(s)")
        self.ax1.set_ylabel("Voltage(volts)")

        self.ax2.set_ylim([-1, 1])
        self.ax2.set_xlim([0, self.tdoa_window_length])
        self.ax2.set_xlabel("Time(s)")
        self.ax2.set_ylabel("Voltage(volts)")

        self.ax3.set_ylim([-1, 1])
        self.ax3.set_xlim([0, self.tdoa_window_length])
        self.ax3.set_xlabel("Time(s)")
        self.ax3.set_ylabel("Voltage(volts)")

        # Subscriber
        rospy.Subscriber("tdoa", Tdoa, self.tdoa_cb)

    def tdoa_cb(self, msg):

        # Subcribe data
        ch1_filtered = np.array(msg.data_ch1)
        ch1_raw = np.array(msg.data_ch1_raw)

        self.threshold = msg.threshold
        self.ch1_avg = msg.ch1_avg
        print(f'ch1_avg : {self.ch1_avg}')
        #print(f'Incoming msg size : {ch1.shape}')
        #print "ch1 shape : ", ch1.shape #(192000,)
        #time_start = rospy.get_time()
        #print "time_start : ", time_start

        # Remove previous data and append new data
        self.plot_array_ch1_filtered = ch1_filtered
        self.plot_array_ch1_raw = ch1_raw

        # Calculate threshold
        #self.threshold_line_ch1 = self.threshold * self.ch1_avg * np.ones(ch1.shape[0])
        #print(f'After remove : {self.plot_array_ch1.shape}')
        #print(f'After append : {self.plot_array_ch1.shape}')


    def animation_frame(self,i):

        print(f'shape of t axis          {self.t_axis.shape}')
        print(f'shape of plot array      {self.plot_array_ch1_filtered.shape}')
        print(f'shape of plot array      {self.plot_array_ch1_raw.shape}')
        # Notice : t_axis & ch1 should be same size

        if self.t_axis.shape[0] == self.plot_array_ch1_filtered.shape[0]:
            print(f'Plot!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self.line[0].set_data(self.t_axis, self.plot_array_ch1_filtered)

            self.line[1].set_data(self.t_axis, self.plot_array_ch1_raw)

            self.line[2].set_data(self.t_axis, self.plot_array_ch1_filtered)
            self.line[3].set_data(self.t_axis, self.plot_array_ch1_raw)

        return self.line


        
    def onShutdown(self):
        rospy.loginfo("PLOT TIME SERIES node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('plot_tdoa')
    node = PLOT()
    animation = FuncAnimation(node.fig, node.animation_frame, interval=500, blit=True)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
