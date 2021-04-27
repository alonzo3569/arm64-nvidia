#!/usr/bin/env python

import rospy
import numpy as np
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData
from std_msgs.msg import Header


class FFT:

        # Static params
        FRAME_ID = "base_link"


	def __init__(self):

            # OnstartUp
            self.chunk = rospy.get_param('~chunk', 1024)
            self.fs = rospy.get_param('~rate', 192000)
            self.overlap = rospy.get_param('~overlap', 0.9)
            #print "Set Format   : ", str(self.format), type(self.format)

            # Initialize
            self.fft_msg = HydrophoneFFTData()
            self.win = np.hanning(self.chunk)
            self.ch1_data_array = np.array([])
            self.ch2_data_array = np.array([])

            self.count = 0
            self.msg_number = 0


            # Subscriber
            rospy.Subscriber("/hydrophone_data", HydrophoneData, self.hydro_cb)

            # Publisher
	    self.pub = rospy.Publisher("/fft", HydrophoneFFTData, queue_size=10)

        def hydro_cb(self, msg):

            # Subcribe data
            ch1 = np.array(msg.data_ch1)
            ch2 = np.array(msg.data_ch2)
            #print "ch1 shape : ", ch1.shape #(192000,)
            #time_start = rospy.get_time()
            #print "time_start : ", time_start

            # Append data to array 
            self.ch1_data_array = np.concatenate((self.ch1_data_array,ch1))
            self.ch2_data_array = np.concatenate((self.ch2_data_array,ch2))
            #print "ch1 data shape : ", self.ch1_data_array.shape[0] #192000

            while self.ch1_data_array.shape[0] >= self.chunk:

                # Normalized, windowed frequencies in data chunk
                spec1 = np.fft.rfft(self.ch1_data_array[:self.chunk] * self.win) #/ CHUNKSZ
                spec2 = np.fft.rfft(self.ch2_data_array[:self.chunk] * self.win) #/ CHUNKSZ
                #print "ch1 chunk 1 size : ", self.ch1_data_array[:self.chunk].shape # (1024,)
                #print "win shape : ", self.win.shape # (1024,)
                #print "spec1 size : ", spec1.shape # (513,)

                # Get magnitude & convert to dB
                psd1 = abs(spec1)
                psd2 = abs(spec2)
                psd1 = 20 * np.log10(psd1)
                psd2 = 20 * np.log10(psd2)
                #print "psd1 : ", psd1.shape #(513,)
                self.fft_msg.fft_ch1.extend(psd1.tolist()) # len(fft_ch1) = 513 
                self.fft_msg.fft_ch2.extend(psd2.tolist())

                # Remove data
                self.ch1_data_array = self.ch1_data_array[int(self.chunk*(1-self.overlap)):]
                self.ch2_data_array = self.ch2_data_array[int(self.chunk*(1-self.overlap)):]

                self.count = self.count + 1

            # Publish fft data
            self.fft_msg.header = Header(frame_id=FFT.FRAME_ID, stamp=rospy.Time.now())
            self.fft_msg.header = Header(stamp=rospy.Time.now())
            self.fft_msg.fs = self.fs
            self.fft_msg.delta_f = self.fs / float(self.chunk)
            self.fft_msg.delta_t = (float(self.chunk) / self.fs)*(1 - self.overlap)
            self.pub.publish(self.fft_msg)
            #print"Shape of msg.fft_ch2 : ", np.array(self.fft_msg.fft_ch2).shape (965466,)

            # clear after publish
            del self.fft_msg.fft_ch1[:]
            del self.fft_msg.fft_ch2[:]


            print "Finish process ", self.count, " chunks of data." # process 1sec data in 0.2sec
            self.count = 0
            self.msg_number = self.msg_number + 1
            print "Publish msg number ", self.msg_number
            #time_end = rospy.get_time() 
            #print "time_end : ", time_end
            #print "Process 1s data spent : ", time_end-time_start, " sec"





        def onShutdown(self):
            rospy.loginfo("FFT node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('fft')
	node = FFT()
        rospy.on_shutdown(node.onShutdown)
        rospy.spin()
