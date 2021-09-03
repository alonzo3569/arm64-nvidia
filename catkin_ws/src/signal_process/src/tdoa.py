#!/usr/bin/env python

import rospy
import math
import numpy as np
from scipy.signal import butter, lfilter, fftconvolve
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
            self.mic_distance = rospy.get_param('~mic_distance', 0.3)
            self.c = rospy.get_param('~sound_speed', 343)
            ##print "Set Format   : ", str(self.format), type(self.format)

            # Initialize
            self.ch1_data_array = np.array([])
            self.ch2_data_array = np.array([])
            self.tdoa_array_size = int(self.tdoa_window_length * self.fs)
            self.msg_count = 0
            self.ch1_volt_avg = 0.0
            self.ch2_volt_avg = 0.0
            self.tdoa_msg = Tdoa()
            self.ch1_data_array_raw = np.array([])
            self.ch2_data_array_raw = np.array([])
            self.time_cutoff = 0.05 # sec (experience from test result)
            self.gcc = False

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

            # Bandpass
            ch1_filtered = self.butter_bandpass_filter(ch1, lowcut=1000, highcut=15000, fs=self.fs)
            ch2_filtered = self.butter_bandpass_filter(ch2, lowcut=1000, highcut=15000, fs=self.fs)

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


        def Iterate(self, event):
            
            ## If hydrophone data is shorter than tdoa_window
            if (self.ch1_data_array.shape[0] < self.tdoa_array_size):
                print"ch1_data_array : ", self.ch1_data_array.shape[0]
                print"tdoa_array_size : ", self.tdoa_array_size
                print("Data shorter than tdoa window")
                return

            if (self.ch1_data_array.shape[0] != self.ch2_data_array.shape[0]):
                print("Size of ch1 and 2 are different")
                return

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
            #print("thres  ch1 :  ", self.threshold * self.ch1_volt_avg)
            #print("thres  ch2 :  ", self.threshold * self.ch2_volt_avg)

            over_voltage_ch1_front, = np.where(index_ch1 < int(self.time_cutoff * self.fs))
            over_voltage_ch2_front, = np.where(index_ch2 < int(self.time_cutoff * self.fs))
            over_voltage_ch1_back,  = np.where(index_ch1 > int((self.tdoa_window_length - self.time_cutoff) * self.fs))
            over_voltage_ch2_back,  = np.where(index_ch2 > int((self.tdoa_window_length - self.time_cutoff) * self.fs))

            print("===================================================")
            print("   How many voltages are larger than threshold?    ")
            print("===================================================")
            print"ch1  : ", len(index_ch1)
            print"ch2  : ", len(index_ch2)
            print("===================================================")
            print(" Does these volts located in the front cutoff area?")
            print("===================================================")
            print"ch1 : ", len(over_voltage_ch1_front)
            print"ch2 : ", len(over_voltage_ch2_front)
            print("===================================================")
            print(" Does these volts located in the back cutoff area?")
            print("===================================================")
            print"ch1 : ", len(over_voltage_ch1_back)
            print"ch2 : ", len(over_voltage_ch2_back)

            if (index_ch1.size == 0 or # over thres volts exist
                index_ch2.size == 0 or 
                len(over_voltage_ch1_front) != 0 or # over thres volts are not in cutoff range 
                len(over_voltage_ch2_front) != 0 or
                len(over_voltage_ch1_back)  != 0 or 
                len(over_voltage_ch2_back)  != 0 ):
                print("Signal below threshold or appear in cutoff range")
                return


            # Calculate time difference
            if self.gcc == True:
                #tau, cc = self.gcc_phat_non_interpol(ch1, ch2, fs=self.fs)
                #tau, cc = self.gcc_phat(ch1, ch2, fs=self.fs)
                tau, cc = self.cc(ch1, ch2, fs=self.fs)
            else:
                tau = self.basic_threshold(index_ch1[0], index_ch2[0])
                #tau, stable_tau = self.stable_threshold(index_ch1[0],
                #                                        index_ch2[0],
                #                                        index_ch1[-1],
                #                                        index_ch2[-1],
                #                                        ch1,
                #                                        ch2)

            # Calculate angle (Assume ch1:left ch2:right)
            limit = self.mic_distance / self.c

            if abs(tau) > limit:
                print("Tau  out of  range.........")

            rad = np.arcsin(self.c * tau / self.mic_distance)
            theta = math.degrees(rad)
            print "==========================="  
            print "Tau    : ",  tau 
            print "theta : ", theta 
            print "==========================="

            # Publish Tdoa msg
            self.tdoa_msg.header = Header(frame_id=TDOA.FRAME_ID, stamp=rospy.Time.now())
            self.tdoa_msg.data_ch1.extend(ch1.tolist())
            self.tdoa_msg.data_ch2.extend(ch2.tolist())

            # Debug
            self.tdoa_msg.data_ch1_raw.extend(ch1_raw.tolist())
            self.tdoa_msg.data_ch2_raw.extend(ch2_raw.tolist())
            if self.gcc == True:
                #self.tdoa_msg.cc.extend(cc.tolist())
                self.tdoa_msg.cc.extend([])

            self.tdoa_msg.threshold = self.threshold
            self.tdoa_msg.ch1_avg = self.ch1_volt_avg
            self.tdoa_msg.ch2_avg = self.ch2_volt_avg
            self.tdoa_msg.basic_tau = tau
            #self.tdoa_msg.stable_tau = stable_tau
            self.tdoa_msg.tdoa_angle = theta
            self.tdoa_msg.mic_distance = self.mic_distance
            self.tdoa_msg.sound_speed = self.c
            self.pub.publish(self.tdoa_msg)

            # Clear after publish
            del self.tdoa_msg.data_ch1[:]
            del self.tdoa_msg.data_ch2[:]

            del self.tdoa_msg.data_ch1_raw[:]
            del self.tdoa_msg.data_ch2_raw[:]
            del self.tdoa_msg.cc[:]

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
            
            return tau, cc

        def gcc_phat(self, sig, refsig, fs=1, max_tau=None, interp=64):
            
            # make sure the length for the FFT is larger or equal than len(sig) + len(refsig)
            n = sig.shape[0] + refsig.shape[0]
        
            # Generalized Cross Correlation Phase Transform
            SIG = np.fft.rfft(sig, n=n)
            REFSIG = np.fft.rfft(refsig, n=n)
            
            R = SIG * np.conj(REFSIG)
        
            cc = np.fft.irfft(R / np.abs(R), n=(interp * n))
        
            max_shift = int(interp * n / 2)
            if max_tau:
                max_shift = np.minimum(int(interp * fs * max_tau), max_shift)
        
            cc = np.concatenate((cc[-max_shift:], cc[:max_shift+1]))
        
            # find max cross correlation index
            shift = np.argmax(np.abs(cc)) - max_shift
        
            tau = shift / float(interp * fs)
            
            return tau, cc

        def basic_threshold(self, index_ch1_start, index_ch2_start):

            # Calculate time difference tau using threshold method
            tau = float(index_ch1_start - index_ch2_start) / self.fs
            return tau

        def find_pks(self, data):

            # init
            iter_index = np.arange(1, len(data)-1, 1)
            locs = np.array([])

            # iter
            for i in iter_index:
                if data[i] > data[i-1] and  data[i] > data[i+1]: # this is peak
                    locs = np.append(locs, i) # output pk value index

            return locs.astype(int)

        def stable_threshold(self, index_ch1_start, 
                                   index_ch2_start, 
                                   index_ch1_end,
                                   index_ch2_end,
                                   filtered_ch1,
                                   filtered_ch2):
        
            # Get mean value from over threshold value
            # Cut
            tmp_ch1 = filtered_ch1[np.arange(index_ch1_start, index_ch1_end+1, 1)]
            tmp_ch2 = filtered_ch2[np.arange(index_ch2_start, index_ch2_end+1, 1)]
        
            # Find peaks
            locs1 = self.find_pks(tmp_ch1)
            locs2 = self.find_pks(tmp_ch2)
            ch1_pks_value = tmp_ch1[locs1]
            ch2_pks_value = tmp_ch2[locs2]
            
            ch1_peak_avg = np.average(ch1_pks_value)
            ch2_peak_avg = np.average(ch2_pks_value)
            
            # Get first stable voltage from each channel as time difference  
            stable_ch1_index_list, = np.where(filtered_ch1 > ch1_peak_avg)
            stable_ch2_index_list, = np.where(filtered_ch2 > ch1_peak_avg)
            
            stable_ch1_index_start = stable_ch1_index_list[0]
            stable_ch2_index_start = stable_ch2_index_list[0]
        
            # Calculate time difference tau using yhh threshold method
            tau = float(index_ch1_start - index_ch2_start) / self.fs
            stable_tau = float(stable_ch1_index_start - stable_ch2_index_start) / self.fs
            #print("       tau : ", tau)
            #print("stable tau : ", stable_tau)
        
            return tau, stable_tau

        def cc(self, ch1, ch2, fs):
            #cc = np.correlate(ch1, ch2, 'full') # np.correlate : to slow
            #cc = np.correlate(ch1, ch2, 'same')

            cc = fftconvolve(ch1, ch2[::-1], 'full')

            index = np.argmax(np.abs(cc))
            tau = float(len(cc)/2 - index)/ fs
            return tau, cc


        def onShutdown(self):
            rospy.loginfo("TDOA node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('tdoa')
	node = TDOA()
        rospy.on_shutdown(node.onShutdown)
        rospy.spin()
