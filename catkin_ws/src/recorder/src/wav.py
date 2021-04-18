#!/usr/bin/env python

import rospy
import wave
import numpy as np
from struct import pack
from ntu_msgs.msg import HydrophoneData
from std_msgs.msg import Int64
from std_srvs.srv import SetBool


class Wav:

    def __init__(self):

        # OnstartUp
        self.channels = rospy.get_param('~channels', 2)
        self.rate = rospy.get_param('~rate', 192000)

        # Initialize wav 
        self.wav = wave.open("voice_test.wav", 'wb')
        self.wav.setnchannels(self.channels)
        self.wav.setsampwidth(4) # 4 byte for 32 bits
        self.wav.setframerate(self.rate)

        # Publisher
        self.hydro_sub = rospy.Subscriber("/hydrophone_data", HydrophoneData, self.hydro_cb)


    def hydro_cb(self, msg):
        # Subscribe data & revert into bits
        ch1 = np.array(msg.data_ch1) * 2**31
        ch2 = np.array(msg.data_ch2) * 2**31
        #print "ch1.shape", ch1.shape

        # Merge both array into interleave mode
        tmp = np.vstack((ch1, ch2))
        #print "frame shape", tmp.shape
        stream = tmp.flatten('F')

        # Change bits(in int) into 32bits binary code
        binary = [pack('<i',int(x)) for x in stream]
        #print "binary : ", binary
        binary = ''.join(binary)
        self.wav.writeframes(binary)


    def onShutdown(self):
        self.wav.close()
        rospy.loginfo("Recorder node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('wave_node')
    Wav()
    rospy.spin()
