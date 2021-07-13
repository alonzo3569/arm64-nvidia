#!/usr/bin/env python

import rospy
import pyaudio
import numpy as np
from ntu_msgs.msg import HydrophoneData
from std_msgs.msg import Header


class Recorder:

        # Static params
        FRAME_ID = "base_link"


	def __init__(self):

            # OnstartUp
            self.format = rospy.get_param('~format', pyaudio.paInt32)
            self.channels = rospy.get_param('~channels', 2) 
            self.rate = rospy.get_param('~rate', 192000) 
            self.chunk = rospy.get_param('~chunk', 19200)
            self.msg_length = rospy.get_param('~msg_length', 1.0)
            print "Set Format   : ", str(self.format), type(self.format)
            print "Set Channels : ", self.channels, type(self.channels)
            print "Set Rate : ", self.rate, type(self.rate)
            print "Set Chunk : ", self.chunk, type(self.chunk)
            print "Set Msg length : ", self.msg_length, type(self.msg_length)

            # Initialize pyaudio

            # Initialize msg
            self.hydro_msg = HydrophoneData()

            # Publisher


        def run(self):
            if ros.Time.now() == 1625922850:
                rate = rospy.Rate(0.1) # 10 Hz
                while not rospy.is_shutdown():
                    print"2 rospy.get_time() : ", rospy.get_time()
                    print"2 rospy.Time.now() : ", rospy.Time.now()
                    rate.sleep()

        def onShutdown(self):
            rospy.loginfo("Recorder node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('recorder_2')
	node = Recorder()
        node.run()
        rospy.on_shutdown(node.onShutdown)
