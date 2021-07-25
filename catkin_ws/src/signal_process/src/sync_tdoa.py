#!/usr/bin/env python

import rospy
import math
import numpy as np
import message_filters
from ntu_msgs.msg import SyncTdoa
from std_msgs.msg import Header


class TDOA:

        # Static params
        FRAME_ID = "base_link"


	def __init__(self):

            # OnstartUp
            #self.tdoa_window_length= rospy.get_param('~tdoa_window_length', 1.0) #sec
            #self.fs = rospy.get_param('~rate', 192000)
            #self.threshold = rospy.get_param('~threshold', 10.0)
            #self.mic_distance = rospy.get_param('~mic_distance', 0.3)
            #self.c = rospy.get_param('~sound_speed', 343)
            ##print "Set Format   : ", str(self.format), type(self.format)

            # Initialize
            self.duckieboat_ch1_time = 0
            self.heron_ch1_time = 0

            # Subscriber
            #rospy.Subscriber("/duckiebaot/sync_tdoa", SyncTdoa, self.duckieboat_cb)
            #rospy.Subscriber("/heron/sync_tdoa", SyncTdoa, self.heron_cb)
            duckieboat_sub = message_filters.Subscriber("/duckieboat/sync_tdoa", SyncTdoa)
            heron_sub = message_filters.Subscriber("/heron/sync_tdoa", SyncTdoa)

            ats = message_filters.ApproximateTimeSynchronizer([duckieboat_sub, heron_sub], queue_size=5, slop=0.1 )
            ats.registerCallback(self.tdoa_cb)

            # Publisher
	    #self.pub = rospy.Publisher("tdoa", Tdoa, queue_size=10)

            # Iterate
            #self.ticks = 0.5
            #rospy.Timer(rospy.Duration(0.5), self.Iterate)

       # def duckieboat_cb(self, msg):

       #     # Subcribe data
       #     self.duckieboat_ch1_time = msg.ch1_time

       # def heron_cb(self, msg):

       #     # Subcribe data
       #     self.heron_ch1_time = msg.ch1_time

        def tdoa_cb(self, msg_duckieboat, msg_heron):
            #self.duckieboat_cb(msg_duckieboat)
            #self.heron_cb(msg_heron)

            self.duckieboat_ch1_time = msg_duckieboat.ch1_time
            self.heron_ch1_time = msg_heron.ch1_time
            print("duckieboat time :", self.duckieboat_ch1_time)
            print("heron      time :", self.heron_ch1_time)
            


        def onShutdown(self):
            rospy.loginfo("TDOA node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('sync_tdoa_sub_node')
	node = TDOA()
        rospy.on_shutdown(node.onShutdown)
        rospy.spin()
