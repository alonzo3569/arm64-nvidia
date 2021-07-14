#!/usr/bin/env python3
'''
Plot ch1 time series data after bandpass filter & threshold
     ch2 time series data after bandpass filter & threshold
     ch1 & ch2 time series data after bandpass filter
     cross correlation result
Msg type : ntu_msgs/Tdoa
Author : logan zhang
'''

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData, Tdoa
from std_msgs.msg import Header


class PLOT:

    # Static params
    FRAME_ID = "base_link"

    def __init__(self):
        # OnstartUp
        self.fs = rospy.get_param('~rate', 192000) # 192000
        self.tdoa_window_length= rospy.get_param('~tdoa_window_length', 1.0) #sec

        # Initialize params
        self.plot_array_ch1 = np.array([])
        self.plot_array_ch2 = np.array([])
        self.cc = np.array([])
        self.t_axis = np.linspace(0, self.tdoa_window_length, int(self.fs * self.tdoa_window_length))
        self.threshold = 0.0
        self.ch1_avg = 0.0
        self.ch2_avg = 0.0
        self.threshold_line_ch1 = np.array([])   
        self.threshold_line_ch2 = np.array([])   

        # Initialize fig
        self.fig, (self.ax1,self.ax2, self.ax3, self.ax4) = plt.subplots(4,1)
        self.line1, = self.ax1.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line2, = self.ax1.plot([], [], color='red', linestyle='-', linewidth=1)
        self.line3, = self.ax2.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line4, = self.ax2.plot([], [], color='red', linestyle='-', linewidth=1)
        self.line5, = self.ax3.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line6, = self.ax3.plot([], [], color='red', linestyle='-', linewidth=1)
        self.line7, = self.ax4.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line8, = self.ax4.plot([], [], color='red', marker='o', markersize=2, markeredgecolor='black', linestyle='')
        self.line = [self.line1, self.line2, self.line3, self.line4, self.line5, self.line6, self.line7, self.line8]

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

        self.ax4.set_ylim([0, 100])
        self.ax4.set_xlim([0, 384000])
        #self.ax4.set_xlim([0, self.tdoa_window_length])
        self.ax4.set_xlabel("Time(s)")
        self.ax4.set_ylabel("CC value")
        #self.ax4.axis('auto')

        # Subscriber
        rospy.Subscriber("tdoa", Tdoa, self.tdoa_cb)

    def tdoa_cb(self, msg):

        # Subcribe data
        ch1 = np.array(msg.data_ch1)
        ch2 = np.array(msg.data_ch2)

        self.cc = np.array(msg.cc)

        self.threshold = msg.threshold
        self.ch1_avg = msg.ch1_avg
        self.ch2_avg = msg.ch2_avg

        # Remove previous data and append new data
        self.plot_array_ch1 = ch1
        self.plot_array_ch2 = ch2

        # Calculate threshold
        self.threshold_line_ch1 = self.threshold * self.ch1_avg * np.ones(ch1.shape[0])
        self.threshold_line_ch2 = self.threshold * self.ch2_avg * np.ones(ch2.shape[0])


    def animation_frame(self,i):

        #print(f'shape of t axis          {self.t_axis.shape}')
        #print(f'shape of plot array      {self.plot_array_ch1.shape}')
        #print(f'shape of threshold line ch1  {self.threshold_line_ch1.shape}')
        # Notice : t_axis & ch1 should be same size

        if self.t_axis.shape[0] == self.plot_array_ch1.shape[0]:
            self.line[0].set_data(self.t_axis, self.plot_array_ch1)
            self.line[1].set_data(self.t_axis, self.threshold_line_ch1)

            self.line[2].set_data(self.t_axis, self.plot_array_ch2)
            self.line[3].set_data(self.t_axis, self.threshold_line_ch1)

            self.line[4].set_data(self.t_axis, self.plot_array_ch1)
            self.line[5].set_data(self.t_axis, self.plot_array_ch2)

            max_cc_index = np.argmax(np.abs(self.cc))
            index_cc_value = self.cc[max_cc_index]
            self.line[6].set_data(np.arange(0,self.cc.shape[0],1), self.cc)
            self.line[7].set_data(max_cc_index, index_cc_value)
            #self.ax4.set_xlim([0, len(self.cc)])
            #self.ax4.set_ylim([-0.05, 0.05])
            #print(f'cc shape  : {self.cc.shape[0]}')
            #print(f'cc t axis : {len(np.arange(0,self.cc.shape[0],1))}')
            print(f'max value index : {max_cc_index}')
            print(f'max value       : {index_cc_value}')

        return self.line


        
    def onShutdown(self):
        rospy.loginfo("PLOT TIME SERIES node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('cc_check')
    node = PLOT()
    animation = FuncAnimation(node.fig, node.animation_frame, interval=500, blit=True)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
