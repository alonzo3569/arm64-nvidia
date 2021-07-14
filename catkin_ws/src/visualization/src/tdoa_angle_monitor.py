#!/usr/bin/env python3
'''
Plot tdoa angle in polar axis
Msg type : ntu_msgs/Tdoa
Author : logan zhang
'''

import rospy
import math
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
        #self.fs = rospy.get_param('~rate', 192000) # 192000

        # Initialize params
        self.tdoa_angle = 0.0
        self.x_vector = 0.0
        self.y_vector = 0.0

        # Initialize fig
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})

        # Subscriber
        rospy.Subscriber("tdoa", Tdoa, self.tdoa_cb)

    def tdoa_cb(self, msg):

        # Subcribe data
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

        # remove previous quiver
        self.ax.clear()

        # Plot configuration
        # r axis (y)
        self.ax.set_ylim([0,1])
        self.ax.set_yticklabels([])      # remove r axis label (remove circle label)
        #self.ax.set_rticks([])          # remove r axis tick (remove circle)
        #self.ax.set_rlabel_position(0)

        # theta axis (x)
        self.ax.set_thetalim(-np.pi, np.pi)
        self.ax.set_xticks(np.linspace(np.pi, -np.pi, 12, endpoint=False)) # xticks = theta ticks
        self.ax.set_theta_direction(-1)
        self.ax.set_theta_zero_location("N")
        #self.ax.grid(False)
        #self.ax.set_title("TDOA heading")

        # plot current quiver
        self.ax.quiver(0, 0, self.x_vector, self.y_vector, color='red', scale = 2, width=0.02)
        self.ax.plot(0, 0, color='red', marker='o', markersize=8)

        
    def onShutdown(self):
        rospy.loginfo("PLOT TIME SERIES node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('plot_tdoa_angle')
    node = PLOT()
    animation = FuncAnimation(node.fig, node.animation_frame, interval=500)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
