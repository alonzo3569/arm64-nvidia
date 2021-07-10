#!/usr/bin/env python

import rospy
import math
import numpy as np
from scipy.signal import butter, lfilter
from numpy.fft import irfft, rfft
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData, Tdoa
from std_msgs.msg import Header


class TDOA:

        # Static params
        FRAME_ID = "base_link"


	def __init__(self):

            # OnstartUp
            self.tdoa_window_length= rospy.get_param('~tdoa_window_length', 1) #sec
            self.fs = rospy.get_param('~rate', 192000)
            self.threshold = rospy.get_param('~threshold', 10.0)
            self.mic_distance = rospy.get_param('~mic_distance', 1.0)
            self.c = rospy.get_param('~sound_speed', 1500)
            ##print "Set Format   : ", str(self.format), type(self.format)

            # Initialize
            #self.fft_msg = HydrophoneFFTData()
            #self.win = np.hanning(self.chunk)
            self.ch1_data_array = np.array([])
            self.tdoa_array_size = int(self.tdoa_window_length * self.fs)
            self.msg_count = 0
            self.ch1_volt_avg = 0.0
            self.tdoa_msg = Tdoa()
            self.ch1_data_array_raw = np.array([])
            self.time_cutoff = 0.001 # sec (experience from test result)



            # Subscriber
            rospy.Subscriber("hydrophone_data", HydrophoneData, self.hydro_cb)

            # Publisher
	    self.pub = rospy.Publisher("tdoa", Tdoa, queue_size=10)

            # Iterate
            #self.ticks = 0.5
            rospy.Timer(rospy.Duration(0.5), self.Iterate)

        def hydro_cb(self, msg):

            # Subcribe data
            ch1 = np.array(msg.data_ch1)
            #print "ch1 shape : ", ch1.shape #(192000,)
            #time_start = rospy.get_time()
            #print "time_start : ", time_start

            # Bandpass
            ch1_filtered = self.butter_bandpass_filter(ch1, lowcut=1000, highcut=15000, fs=self.fs)

            print "ch1_raw      : ", ch1[100] #(192000,)
            print "ch1_filtered : ", ch1_filtered[100] #(192000,)

            # Calculate avg voltage as threshold
            cur_avg_ch1 = np.average(abs(ch1_filtered))
            self.ch1_volt_avg = (self.ch1_volt_avg * self.msg_count + cur_avg_ch1) / (self.msg_count + 1)
            self.msg_count = self.msg_count + 1
           
            # Append data to array 
            self.ch1_data_array = np.concatenate((self.ch1_data_array,ch1_filtered))
            self.ch1_data_array_raw = np.concatenate((self.ch1_data_array_raw,ch1))


        def Iterate(self, event):
            
            print "ch1 data buffer before iterate : ", self.ch1_data_array.shape[0] #192000
            
            ## If hydrophone data is longer than tdoa_window
            if self.ch1_data_array.shape[0] >= self.tdoa_array_size:

                # Extract tdoa_window_length data from hydro array
                ch1 = self.ch1_data_array[:self.tdoa_array_size]
                self.ch1_data_array = self.ch1_data_array[self.tdoa_array_size:]

                # Debug
                ch1_raw = self.ch1_data_array_raw[:self.tdoa_array_size]
                self.ch1_data_array_raw = self.ch1_data_array_raw[self.tdoa_array_size:]

                # Publish Tdoa msg
                self.tdoa_msg.header = Header(frame_id=TDOA.FRAME_ID, stamp=rospy.Time.now())
                self.tdoa_msg.data_ch1.extend(ch1.tolist())

                # Debug
                self.tdoa_msg.data_ch1_raw.extend(ch1_raw.tolist())

                self.tdoa_msg.threshold = self.threshold
                self.tdoa_msg.ch1_avg = self.ch1_volt_avg
                self.pub.publish(self.tdoa_msg)

                # Clear after publish
                del self.tdoa_msg.data_ch1[:]

                del self.tdoa_msg.data_ch1_raw[:]


            print "ch1 data buffer after iterate : ", self.ch1_data_array.shape[0] #0

        def butter_bandpass(self, lowcut, highcut, fs, order):
            nyq = 0.5 * fs
            low = lowcut / nyq
            high = highcut / nyq
            b, a = butter(order, [low, high], btype='bandpass')
            return b, a
        
        def butter_bandpass_filter(self, data, lowcut, highcut, fs, order=5):
            b, a = self.butter_bandpass(lowcut, highcut, fs, order=order)
            y = lfilter(b, a, data)
            return y



        def onShutdown(self):
            rospy.loginfo("TDOA node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('tdoa')
	node = TDOA()
        rospy.on_shutdown(node.onShutdown)
        rospy.spin()
