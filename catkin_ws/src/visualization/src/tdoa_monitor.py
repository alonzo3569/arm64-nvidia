#!/usr/bin/env python3
'''
Plot ch1 time series data after bandpass filter & threshold
     ch2 time series data after bandpass filter & threshold
     ch1 & ch2 time series data after bandpass filter
Msg type : ntu_msgs/Tdoa
Author : logan zhang
'''

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData, Tdoa
from std_msgs.msg import Header
from qbstyles import mpl_style
import math


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
        self.t_axis = np.linspace(0, self.tdoa_window_length, int(self.fs * self.tdoa_window_length))
        self.threshold = 0.0
        self.ch1_avg = 0.0
        self.ch2_avg = 0.0
        self.threshold_line_ch1 = np.array([])   
        self.threshold_line_ch2 = np.array([])  
        # angle
        self.tdoa_angle = 0.0
        self.x_vector = 0.0
        self.y_vector = 0.0
        
        # matplotlib styling
        mpl_style(dark = True)

        # Initialize fig
        
        self.fig, ((self.ax1,self.ax2), (self.ax3, self.ax4)) = plt.subplots(2,2, figsize=(15, 10))
        self.ax4 = plt.subplot(2, 2, 4, projection='polar')
        self.fig.canvas.set_window_title('TDOA Monitor')
        #self.fig.tight_layout() # auto reduce the space between window and plot
        self.line1, = self.ax1.plot([], [], color='C8', linestyle='-', linewidth=0.5, label='Ch1 voltage')
        self.line2, = self.ax1.plot([], [], color='C7', linestyle='-', linewidth=1.0, label='Ch1 threshold')
        self.line3, = self.ax2.plot([], [], color='C6', linestyle='-', linewidth=0.5, label='Ch2 voltage')
        self.line4, = self.ax2.plot([], [], color='C7', linestyle='-', linewidth=1.0, label='Ch2 threshold')
        self.line5, = self.ax3.plot([], [], color='C8', linestyle='-', linewidth=1, label='Ch1 voltage')
        self.line6, = self.ax3.plot([], [], color='C6', linestyle='-', linewidth=1, label='Ch2 voltage')
        self.line7 = self.ax4.quiver(0, 0, 0, 1, color='C10', scale = 2, width=0.02)
        self.line = [self.line1, self.line2, self.line3, self.line4, self.line5, self.line6, self.line7]
        self.ax1.set_title('Channel 1')
        self.ax1.set_ylim([-1, 1])
        self.ax1.set_xlim([0, self.tdoa_window_length])
        self.ax1.set_xlabel("Time (s)")
        self.ax1.set_ylabel("Voltage (volts)")
        self.ax1.legend(loc='upper right')

        self.ax2.set_title('Channel 2')
        self.ax2.set_ylim([-1, 1])
        self.ax2.set_xlim([0, self.tdoa_window_length])
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Voltage (volts)")
        self.ax2.legend(loc='upper right')

        self.ax3.set_title('Channel 1 + Channel 2')
        self.ax3.set_ylim([-1, 1])
        self.ax3.set_xlim([0, self.tdoa_window_length])
        self.ax3.set_xlabel("Time (s)")
        self.ax3.set_ylabel("Voltage (volts)")
        self.ax3.legend(loc='upper right')
        #self.ax3.tick_params(axis='x', labelrotation=35)
            
        self.ax4.plot(0, 0, color='C10', marker='o', markersize=8)

        # Plot configuration
        # r axis (y)
        self.ax4.set_ylim([0,1])
        self.ax4.set_yticklabels([])      # remove r axis label (remove circle label)
        #self.ax.set_rticks([])          # remove r axis tick (remove circle)
        #self.ax.set_rlabel_position(0)

        # theta axis (x)
        self.ax4.set_thetalim(-np.pi, np.pi)
        self.ax4.set_xticks(np.linspace(np.pi, -np.pi, 12, endpoint=False)) # xticks = theta ticks
        self.ax4.set_theta_direction(-1)
        self.ax4.set_theta_zero_location("N")
        #self.ax.grid(False)
        #self.ax.set_title("TDOA heading")
        self.ax4.set_title('TDOA (degree)', pad = 30)

        plt.subplots_adjust(wspace = 0.5, hspace = 0.6)

        # Subscriber
        rospy.Subscriber("tdoa", Tdoa, self.tdoa_cb)

    def tdoa_cb(self, msg):

        # Subcribe tdoa time series data
        ch1 = np.array(msg.data_ch1)
        ch2 = np.array(msg.data_ch2)


        self.threshold = msg.threshold
        self.ch1_avg = msg.ch1_avg
        self.ch2_avg = msg.ch2_avg

        # Remove previous data and append new data
        self.plot_array_ch1 = ch1
        self.plot_array_ch2 = ch2

        # Calculate threshold
        self.threshold_line_ch1 = self.threshold * self.ch1_avg * np.ones(ch1.shape[0])
        self.threshold_line_ch2 = self.threshold * self.ch2_avg * np.ones(ch2.shape[0])
                   
        # Subcribe tdoa angle data
        self.tdoa_angle = msg.tdoa_angle

        theta = self.tdoa_to_polar(self.tdoa_angle)
        print(f'polar angle : {theta}')

        self.x_vector = math.cos(math.radians(theta))
        self.y_vector = math.sin(math.radians(theta))

    def tdoa_to_polar(self, theta):
        '''
        tdoa      90           polar      90         
                   |                       |
          180      |      0       180      |       0
             ----- A -----           ----- A -----    
         -180      |      0       180      |       0        
                   |                       |            
                  -90                     270        
        '''  
        if theta > 90.0 and theta < 180.0:
            theta = -theta + 450.0
        else:
            theta = -theta + 90

        return theta

    def animation_frame(self,i):

        #print(f'shape of t axis          {self.t_axis.shape}')
        #print(f'shape of plot array      {self.plot_array_ch1.shape}')
        #print(f'shape of threshold line ch1  {self.threshold_line_ch1.shape}')
        # Notice : t_axis & ch1 should be same size

        if self.t_axis.shape[0] == self.plot_array_ch1.shape[0]:
            self.line[0].set_data(self.t_axis, self.plot_array_ch1)
            self.line[1].set_data(self.t_axis, self.threshold_line_ch1)

            self.line[2].set_data(self.t_axis, self.plot_array_ch2)
            self.line[3].set_data(self.t_axis, self.threshold_line_ch2)

            self.line[4].set_data(self.t_axis, self.plot_array_ch1)
            self.line[5].set_data(self.t_axis, self.plot_array_ch2)

            # plot current quiver
            self.line[6].set_UVC(np.array([self.x_vector]), np.array([self.y_vector]))
    
        return self.line

        
    def onShutdown(self):
        rospy.loginfo("PLOT TIME SERIES node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('tdoa_signal_monitor')
    node = PLOT()
    animation = FuncAnimation(node.fig, node.animation_frame, interval=500, blit=True)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
