#!/usr/bin/env python
'''
Tdoa threshold method using ch1/ch2
Msg type : ntu_msgs/SyncTdoa
Author : logan zhang
'''
import rospy
import pyaudio
import numpy as np
from scipy.signal import butter, lfilter
from ntu_msgs.msg import HydrophoneData, SyncTdoa
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
            self.threshold = rospy.get_param('~threshold', 10.0)
            print "Set Format   : ", str(self.format), type(self.format)
            print "Set Channels : ", self.channels, type(self.channels)
            print "Set Rate : ", self.rate, type(self.rate)
            print "Set Chunk : ", self.chunk, type(self.chunk)
            print "Set Msg length : ", self.msg_length, type(self.msg_length)

            # Initialize pyaudio
            # Device id : 0(xps), 6(focusrite)
            p = pyaudio.PyAudio()
            self.stream = p.open(format=self.format,
                            channels=self.channels,
                            rate=self.rate,
                            input=True,
                            input_device_index=4,
                            frames_per_buffer=self.chunk)

            # Initialize msg
            self.tdoa_msg = SyncTdoa()
            self.msg_count = 0
            self.ch1_volt_avg = 0.0
            self.ch2_volt_avg = 0.0
            self.pre_time_ch1 = 0.0
            self.pre_time_ch2 = 0.0
            self.wait = 5.0 # sec
            self.time_cutoff = self.chunk * 0.1


            # Publisher
	    #self.pub = rospy.Publisher("/hydrophone_data", HydrophoneData, queue_size=10)
	    self.pub = rospy.Publisher("/sync_tdoa", SyncTdoa, queue_size=10)


        def run(self):
            while not rospy.is_shutdown():

                # Read data stream, if chunk size is too small, data will be overwrite by new data
                data = self.stream.read(self.chunk, exception_on_overflow = False)

                # Convert string to numpy array
                msg_begin_time = rospy.Time.now().to_nsec()  # type 'int'
                data_array = np.fromstring(data, dtype='int32')

                # Deinterleave, select 1 channel
                ch_1 = data_array[0::self.channels] / 2.0**31 
                #ch_2 = data_array[1::self.channels] / 2.0**31

                # Bandpass
                ch1_filtered = self.butter_bandpass_filter(ch_1, lowcut=1000, highcut=15000, fs=self.rate)
                #ch2_filtered = self.butter_bandpass_filter(ch_2, lowcut=1000, highcut=15000, fs=self.rate)

                # Threshold tdoa
                cur_avg_ch1 = np.average(abs(ch1_filtered))
                #cur_avg_ch2 = np.average(abs(ch2_filtered))
                self.ch1_volt_avg = (self.ch1_volt_avg * self.msg_count + cur_avg_ch1) / (self.msg_count + 1)
                #self.ch2_volt_avg = (self.ch2_volt_avg * self.msg_count + cur_avg_ch2) / (self.msg_count + 1)
                self.msg_count = self.msg_count + 1

                index1, = np.where(ch1_filtered > self.ch1_volt_avg * self.threshold)
                #index2, = np.where(ch2_filtered > self.ch2_volt_avg * self.threshold)

                over_voltage_ch1_front, = np.where(index1 < int(self.time_cutoff))
                #over_voltage_ch2_front, = np.where(index2 < int(self.time_cutoff))
                over_voltage_ch1_back,  = np.where(index1 > int(self.chunk - self.time_cutoff))
                #over_voltage_ch2_back,  = np.where(index2 > int(self.chunk - self.time_cutoff))


                if (len(over_voltage_ch1_front) != 0 or # over thres volts are not in cutoff range 
                    len(over_voltage_ch1_back)  != 0): 
                    print("Signal appear in cutoff range")
                    continue

                time = rospy.get_time()

                peak_time_ch1, self.pre_time_ch1 = self.wait_for_peak(time, self.pre_time_ch1, index1, msg_begin_time, self.rate, self.wait)
                #peak_time_ch2, self.pre_time_ch2 = self.wait_for_peak(time, self.pre_time_ch2, index2, msg_begin_time, self.rate, self.wait)

                # Don't publish if there's no peak in both channel
                print"peak ch1 : ", peak_time_ch1
                #print"peak ch2 : ", peak_time_ch2
                #print"pre  ch1 : ", self.pre_time_ch1
                #print"pre  ch2 : ", self.pre_time_ch2
                if (peak_time_ch1 == 0):#  and
                    #peak_time_ch2 == 0):
                    print"No peaks found, not publishing...."
                    continue

                # Publish msg
                self.tdoa_msg.header = Header(frame_id=Recorder.FRAME_ID, stamp=rospy.Time.now())
                self.tdoa_msg.ch1_time = peak_time_ch1
                #self.tdoa_msg.ch2_time = peak_time_ch2

                self.tdoa_msg.data_ch1.extend(ch1_filtered.tolist())
                #self.tdoa_msg.data_ch2.extend(ch2_filtered.tolist())

                self.tdoa_msg.threshold = self.threshold
                self.tdoa_msg.ch1_avg = self.ch1_volt_avg
                #self.tdoa_msg.ch2_avg = self.ch2_volt_avg

                self.pub.publish(self.tdoa_msg)

                # Clear after publish
                del self.tdoa_msg.data_ch1[:]
                #del self.tdoa_msg.data_ch2[:]
                
                #rospy.loginfo("Publish %i samples of data", len(self.hydro_msg.data_ch1))

        def get_peak_time(self, time_start, index, fs):
            
            delta_t = float(index) / fs # sec
            epoch = int(delta_t * 1000000000)
            peak_time = time_start + epoch
            print "delta_t       : ", delta_t
            print "delta epoch   : ", epoch
            print "time final    : ", peak_time
            return peak_time

        def wait_for_peak(self, time, pre_time, index, msg_start_time, fs, wait):

            # Quit if time between previous peak is less than 2 sec
            if time - pre_time < wait:
                print"Wait for ", wait, " second"
                return 0, pre_time 

            # Quit if no peak found
            if index.size == 0: 
                print"No peaks found in current buffer"
                return 0,0 

            # Calculate peak time
            print"Output peak"
            peak_time = self.get_peak_time(msg_start_time, index[0], fs)
            pre_time = rospy.get_time()
            return peak_time, pre_time

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
            rospy.loginfo("Recorder node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('tdoa_recorder')
	node = Recorder()
        node.run()
        rospy.on_shutdown(node.onShutdown)
