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
    PLOT_TIME = 5

    def __init__(self):
        # OnstartUp
        self.fs = rospy.get_param('~rate', 192000) # 192000

        # Initialize fig
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3,1)
        self.line1, = self.ax1.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line2, = self.ax2.plot([], [], color='red', linestyle='-', linewidth=1)
        self.line3, = self.ax3.plot([], [], color='blue', linestyle='-', linewidth=1)
        self.line4, = self.ax3.plot([], [], color='red', linestyle='-', linewidth=1)
        self.line5, = self.ax3.plot([], [], color='black', linestyle='-', linewidth=1)
        self.line = [self.line1, self.line2, self.line3, self.line4, self.line5]
        #self.line, = self.ax.plot([], [], color='red', marker='o', markersize=10, markeredgecolor='black', linestyle='')
        #self.ax.set_label_position()
        #self.ax.axis('auto')

        self.ax1.set_ylim([-1, 1])
        self.ax1.set_xlim([0, PLOT.PLOT_TIME])
        self.ax1.set_xlabel("Time(s)")
        self.ax1.set_ylabel("Voltage(volts)")

        self.ax2.set_ylim([-1, 1])
        self.ax2.set_xlim([0, PLOT.PLOT_TIME])
        self.ax2.set_xlabel("Time(s)")
        self.ax2.set_ylabel("Voltage(volts)")

        self.ax3.set_ylim([-1, 1])
        self.ax3.set_xlim([0, PLOT.PLOT_TIME])
        self.ax3.set_xlabel("Time(s)")
        self.ax3.set_ylabel("Voltage(volts)")


        self.t_axis = np.linspace(0, PLOT.PLOT_TIME, self.fs * PLOT.PLOT_TIME)
        self.threshold = 0.1 * np.ones(self.fs * PLOT.PLOT_TIME)

        # Initialize params
        self.plot_array_ch1 = np.zeros(int(PLOT.PLOT_TIME * self.fs))
        self.plot_array_ch2 = np.zeros(int(PLOT.PLOT_TIME * self.fs))


        # Subscriber
        rospy.Subscriber("/hydrophone_data", HydrophoneData, self.hydro_cb)

    def hydro_cb(self, msg):

        # Subcribe data
        ch1 = np.array(msg.data_ch1)
        ch2 = np.array(msg.data_ch2)
        print(f'Incoming msg size : {ch1.shape}')
        #print "ch1 shape : ", ch1.shape #(192000,)
        #time_start = rospy.get_time()
        #print "time_start : ", time_start

        # Remove previous data and append new data
        self.plot_array_ch1 = np.concatenate((self.plot_array_ch1[ch1.shape[0]:], ch1))
        self.plot_array_ch2 = np.concatenate((self.plot_array_ch2[ch2.shape[0]:], ch2))

        print(f'After remove : {self.plot_array_ch1.shape}')
        print(f'After append : {self.plot_array_ch1.shape}')


    def animation_frame(self,i):

        print(f'shape of t axis {self.t_axis.shape}')
        print(f'shape of plot array {self.plot_array_ch1.shape}')

        self.line[0].set_data(self.t_axis, self.plot_array_ch1)
        self.line[1].set_data(self.t_axis, self.plot_array_ch2)
        self.line[2].set_data(self.t_axis, self.plot_array_ch1)
        self.line[3].set_data(self.t_axis, self.plot_array_ch2)
        self.line[4].set_data(self.t_axis, self.threshold)

        return self.line


        
    def onShutdown(self):
        rospy.loginfo("PLOT TIME SERIES node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('plot_time_series')
    node = PLOT()
    animation = FuncAnimation(node.fig, node.animation_frame, interval=500, blit=True)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
