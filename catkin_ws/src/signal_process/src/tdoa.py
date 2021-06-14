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
            self.tdoa_window_length= rospy.get_param('~tdoa_window_length', 1.0) #sec
            self.fs = rospy.get_param('~rate', 192000)
            self.threshold = rospy.get_param('~threshold', 10.0)
            self.mic_distance = rospy.get_param('~mic_distance', 1.0)
            self.c = rospy.get_param('~sound_speed', 1500)
            ##print "Set Format   : ", str(self.format), type(self.format)

            # Initialize
            #self.fft_msg = HydrophoneFFTData()
            #self.win = np.hanning(self.chunk)
            self.ch1_data_array = np.array([])
            self.ch2_data_array = np.array([])
            self.tdoa_array_size = int(self.tdoa_window_length * self.fs)
            self.msg_count = 0
            self.ch1_volt_avg = 0.0
            self.ch2_volt_avg = 0.0
            self.tdoa_msg = Tdoa()
            self.ch1_data_array_raw = np.array([])
            self.ch2_data_array_raw = np.array([])
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
            ch2 = np.array(msg.data_ch2)
            #print "ch1 shape : ", ch1.shape #(192000,)
            #time_start = rospy.get_time()
            #print "time_start : ", time_start

            # Bandpass
            #ch1_filtered = self.butter_bandpass_filter(ch1, lowcut=500, highcut=2000, fs=self.fs)
            ch1_filtered = ch1
            ch2_filtered = ch2
            #ch2_filtered = self.butter_bandpass_filter(ch2, lowcut=500, highcut=2000, fs=self.fs)

            # Calculate avg voltage as threshold
            cur_avg_ch1 = np.average(abs(ch1_filtered))
            cur_avg_ch2 = np.average(abs(ch2_filtered))
            self.ch1_volt_avg = (self.ch1_volt_avg * self.msg_count + cur_avg_ch1) / (self.msg_count + 1)
            self.ch2_volt_avg = (self.ch2_volt_avg * self.msg_count + cur_avg_ch2) / (self.msg_count + 1)
            self.msg_count = self.msg_count + 1
            
            # Append data to array 
            self.ch1_data_array = np.concatenate((self.ch1_data_array,ch1_filtered))
            self.ch2_data_array = np.concatenate((self.ch2_data_array,ch2_filtered))
            self.ch1_data_array_raw = np.concatenate((self.ch1_data_array_raw,ch1))
            self.ch2_data_array_raw = np.concatenate((self.ch2_data_array_raw,ch2))
            #print "ch1 data shape : ", self.ch1_data_array.shape[0] #192000


        def Iterate(self, event):
            
            print "ch1 data buffer before iterate : ", self.ch1_data_array.shape[0] #192000
            
            ## If hydrophone data is longer than tdoa_window
            if self.ch1_data_array.shape[0] >= self.tdoa_array_size and self.ch1_data_array.shape[0] == self.ch2_data_array.shape[0]:

                # Extract tdoa_window_length data from hydro array
                ch1 = self.ch1_data_array[:self.tdoa_array_size]
                ch2 = self.ch2_data_array[:self.tdoa_array_size]
                self.ch1_data_array = self.ch1_data_array[self.tdoa_array_size:]
                self.ch2_data_array = self.ch2_data_array[self.tdoa_array_size:]

                # Debug
                ch1_raw = self.ch1_data_array_raw[:self.tdoa_array_size]
                ch2_raw = self.ch2_data_array_raw[:self.tdoa_array_size]
                self.ch1_data_array_raw = self.ch1_data_array_raw[self.tdoa_array_size:]
                self.ch2_data_array_raw = self.ch2_data_array_raw[self.tdoa_array_size:]

                # Check threshold
                index_ch1, = np.where(ch1 > self.threshold * self.ch1_volt_avg)
                index_ch2, = np.where(ch2 > self.threshold * self.ch2_volt_avg)

                index_ch1_number, = np.where(index_ch1 > int(self.time_cutoff * self.fs))
                index_ch2_number, = np.where(index_ch2 > int(self.time_cutoff * self.fs))
                print"Size of index : ", index_ch1.size
                print"index : ", index_ch1
                print"cutoff index : ", int(self.time_cutoff * self.fs)
                print"index_ch1_number : ", index_ch1_number.size
                if index_ch1.size != 0 and index_ch2.size != 0 and index_ch1_number.size != 0 and index_ch2_number.size != 0:

                    print "index ch1 voltage : ", ch1[index_ch1[0]]  
                    print "threshold ch1 voltage : ", self.threshold * self.ch1_volt_avg  

                    # GCC
                    tau, cc = self.gcc_phat_non_interpol(ch1, ch2, fs=self.fs)

                    # Calculate angle (Assume ch1:left ch2:right)
                    limit = self.mic_distance / self.c
                    if abs(tau) > limit:
                        return
                    print("Hi")
                    rad = np.arcsin(self.c * tau / self.mic_distance)
                    theta = math.degrees(rad)

                    # Publish Tdoa msg
                    self.tdoa_msg.header = Header(frame_id=TDOA.FRAME_ID, stamp=rospy.Time.now())
                    self.tdoa_msg.data_ch1.extend(ch1.tolist())
                    self.tdoa_msg.data_ch2.extend(ch2.tolist())

                    # Debug
                    self.tdoa_msg.data_ch1_raw.extend(ch1_raw.tolist())
                    self.tdoa_msg.data_ch2_raw.extend(ch2_raw.tolist())

                    self.tdoa_msg.threshold = self.threshold
                    self.tdoa_msg.ch1_avg = self.ch1_volt_avg
                    self.tdoa_msg.ch2_avg = self.ch2_volt_avg
                    self.pub.publish(self.tdoa_msg)

                    # Clear after publish
                    del self.tdoa_msg.data_ch1[:]
                    del self.tdoa_msg.data_ch2[:]

                    del self.tdoa_msg.data_ch1_raw[:]
                    del self.tdoa_msg.data_ch2_raw[:]

            # TDOA debug msg
            # ch1_filtered, ch2 time series
            # threshold value line
            # first threshold index
            # time difference



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


        def gcc_phat_non_interpol(self, sig, refsig, fs):
        
            # Generalized Cross Correlation Phase Transform
            SIG = np.fft.rfft(sig)
            REFSIG = np.fft.rfft(refsig)
            
            
            R = SIG * np.conj(REFSIG)
        
            cc = np.fft.irfft(R / np.abs(R))
        
            max_shift = int(sig.shape[0] / 2)
        
            cc = np.concatenate((cc[-max_shift:], cc[:max_shift+1]))
        
            # find max cross correlation index
            shift = np.argmax(np.abs(cc)) - max_shift
        
            tau = shift / float(fs)
            print(tau)
            
            return tau, cc


        def onShutdown(self):
            rospy.loginfo("TDOA node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('tdoa')
	node = TDOA()
        rospy.on_shutdown(node.onShutdown)
        rospy.spin()
