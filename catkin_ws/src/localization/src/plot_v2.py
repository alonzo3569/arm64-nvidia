#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ntu_msgs.msg import Kalman
from turtlesim.msg import Pose

class PLOT:

    # Static params

    def __init__(self):
        # OnstartUp
        #self.fs = rospy.get_param('~rate', 192000) # 192000
        self.gt_x = []
        self.gt_y = []
        self.gps_x = []
        self.gps_y = []
        self.kalman_x = []
        self.kalman_y = []
        self.start = rospy.get_time()

        # Initialize fig
        self.fig, self.ax = plt.subplots() #figsize=(10,3)
        #self.im = self.ax.scatter(self.gt_x, self.gt_y)
        #self.ax.set_label_position()
        self.ax.axis('auto')
        #self.ax.set_ylim([0, 20])
        #self.ax.set_xlabel("Time(s)")
        #self.ax.set_ylabel("Frequency(kHz)")

        # Initialize params

        # Subscriber
        rospy.Subscriber("/KALMAN", Kalman, self.kalman_cb)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_cb)



    def kalman_cb(self, msg):
        self.gps_x.extend(msg.gps_x)
        self.gps_y.extend(msg.gps_y)
        self.kalman_x.extend(msg.kalman_x)
        self.kalman_y.extend(msg.kalman_y)

    def pose_cb(self, msg):
        self.gt_x.append(msg.x)
        self.gt_y.append(msg.y)



    def animation_frame(self,i):
        print(f'len of x : {len(self.gt_x)}')
        print(f'len of y : {len(self.gt_y)}')
        if len(self.gt_x) == len(self.gt_y):
            self.ax.scatter(self.gt_x, self.gt_y)
            #self.im.set_offsets([self.gps_x, self.gps_y])
            #time_end = int(rospy.get_time() - self.animation_start)
            #time_start = time_end - PLOT.PLOT_FFT_TIME
            #self.im.set_extent([0,5,0,96])
            return [self.ax]


        
    def onShutdown(self):
        rospy.loginfo("Plot node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('plot')
    node = PLOT()
    animation = FuncAnimation(node.fig, node.animation_frame, interval=500, blit=True)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
