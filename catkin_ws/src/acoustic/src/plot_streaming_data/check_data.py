#!/usr/bin/env python3
# import python library
import numpy as np 
import time

# import ROS library
import rospy
from ntu_msgs.msg import HydrophoneData

class check_data_node:
    def __init__(self):
        # member variables for HydrophoneData msg
        self.data_ch1 = np.array([])
        self.data_ch2 = np.array([])
        self.fs = 96000
        self.length = 1024

        rospy.Subscriber("/get_sound_data_for2i2/hydrophone_data", HydrophoneData, self.push)
    
    # define a callback function to push hydrophone data to container 
    def push(self, data):
        self.data_ch1 = np.concatenate((self.data_ch1, data.data_ch1))
        self.data_ch2 = np.concatenate((self.data_ch2, data.data_ch2))
        self.fs = data.fs
        self.length = data.length
        number = self.fs*10
        if(len(self.data_ch1) > number):
            self.data_ch1 = self.data_ch1[-number:]
            self.data_ch2 = self.data_ch2[-number:]

def main():
    rospy.init_node("check_data_node", anonymous=True)
    node = check_data_node()
    while not rospy.is_shutdown():
        if(len(node.data_ch1)):
            print("lenght of the data ch1: ", len(node.data_ch1))
            print("the max of the data ch1: ", np.max(node.data_ch1))
            print("the min of the data ch1: ", np.min(node.data_ch1))
        time.sleep(5)
if __name__ == "__main__":
    main()
