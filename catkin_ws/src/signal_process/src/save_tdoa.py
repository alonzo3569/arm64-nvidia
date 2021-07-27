#!/usr/bin/env python

import rospy
import numpy as np
from struct import pack
from ntu_msgs.msg import Tdoa
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
import os
import time
import yaml


class SaveTdoa:

    def __init__(self):

        # OnstartUp
        self.path = rospy.get_param('~path', "/root/SSD")

        # Initialize
        self.msg_count = 0
        try:
            os.mkdir(self.path)

        except OSError:
            print "tdoa directory already exist!"

        #self.filename = time.strftime("%Y%m%d-%H%M%S")

        # Publisher
        self.tdoa_sub = rospy.Subscriber("tdoa", Tdoa, self.save_tdoa_cb)


    def save_tdoa_cb(self, msg):

        # Create msg folder (1, 2, 3, 4, 5....)
        self.msg_count += 1 
        file_path = self.path + str(self.msg_count)
        os.mkdir(file_path)


        # Subscribe data
        ch1_filtered = np.array(msg.data_ch1)
        ch2_filtered = np.array(msg.data_ch2)

        ch1_raw = np.array(msg.data_ch1)
        ch2_raw = np.array(msg.data_ch2)

        threshold = msg.threshold

        ch1_avg = msg.ch1_avg
        ch2_avg = msg.ch2_avg

        tau = msg.basic_tau
        tdoa_angle = msg.tdoa_angle
        mic_distance = msg.mic_distance
        sound_speed = msg.sound_speed

        # Create csv file
        np.savetxt(file_path + '/filtered_ch1.csv', ch1_filtered, delimiter=",")
        np.savetxt(file_path + '/filtered_ch2.csv', ch1_filtered, delimiter=",")

        np.savetxt(file_path + '/raw_ch1.csv', ch1_raw, delimiter=",")
        np.savetxt(file_path + '/raw_ch2.csv', ch2_raw, delimiter=",")

        # Create yaml file
        yaml_dict = {}
        yaml_dict['mic_distance'] = mic_distance
        yaml_dict['sound_speed'] = sound_speed
        yaml_dict['tdoa_angle'] = tdoa_angle
        yaml_dict['tau'] = tau
        yaml_dict['thresdhold'] = threshold
        yaml_dict['ch1_avg'] = ch1_avg
        yaml_dict['ch2_avg'] = ch2_avg
        yaml_dict['vehicle'] = ['Duckieboat']  # pyyaml bug
        #print "yaml dictionary = ", yaml_dict
        #print "yaml dictionary type ", type(yaml_dict)

        with open(file_path + '/config.yaml', 'w') as file:
            documents = yaml.dump(yaml_dict, file)


    def onShutdown(self):
        rospy.loginfo("Recorder node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('save_tdoa_node')
    SaveTdoa()
    rospy.spin()
