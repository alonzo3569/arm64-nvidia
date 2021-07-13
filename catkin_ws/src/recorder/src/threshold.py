#!/usr/bin/env python

import rospy
import pyaudio
import numpy as np
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
            #self.hydro_msg = HydrophoneData()
            self.tdoa_msg = SyncTdoa()
            self.msg_count = 0
            self.ch1_volt_avg = 0.0
            self.ch2_volt_avg = 0.0
            self.pre_time = 0.0
            self.wait = False


            # Publisher
	    #self.pub = rospy.Publisher("/hydrophone_data", HydrophoneData, queue_size=10)
	    self.pub = rospy.Publisher("/sync_tdoa", SyncTdoa, queue_size=10)


        def run(self):
            while not rospy.is_shutdown():

                # Read data stream, if chunk size is too small, data will be overwrite by new data
                # type(data) : str
                data = self.stream.read(self.chunk, exception_on_overflow = False)

                # Convert string to numpy array
                # data array element type : int32
                time_stamp = rospy.Time.now().to_nsec()  # type 'int'
                print"start : ", time_stamp
                print"type  : ", type(time_stamp)
                data_array = np.fromstring(data, dtype='int32')

                # Deinterleave, select 1 channel
                # ch_1, ch_2 type : float64
                # size of ch_1, ch_2 : self.chunk=19200, shape : (19200,)
                ch_1 = data_array[0::self.channels] / 2.0**31 
                ch_2 = data_array[1::self.channels] / 2.0**31
                #print "ch_1 shape", ch_1.shape

                # Threshold tdoa
                cur_avg_ch1 = np.average(abs(ch_1))
                cur_avg_ch2 = np.average(abs(ch_2))
                self.ch1_volt_avg = (self.ch1_volt_avg * self.msg_count + cur_avg_ch1) / (self.msg_count + 1)
                self.ch2_volt_avg = (self.ch2_volt_avg * self.msg_count + cur_avg_ch2) / (self.msg_count + 1)
                self.msg_count = self.msg_count + 1

                index1, = np.where(ch_1 > self.ch1_volt_avg * self.threshold)
                index2, = np.where(ch_2 > self.ch2_volt_avg * self.threshold)

                time = rospy.get_time()

                #self.function(time, pre_time, wait)
                if time - self.pre_time > 2.0 and self.wait == True: #sec
                    print"reset"
                    self.wait = False

                # Quit if no peak found
                if index1.size == 0 or self.wait == True:
                    print"quit"
                    continue

                # Calculate peak time
                peak_time_ch1 = self.get_peak_time(time_stamp, index1[0], self.rate)
                self.wait = True
                self.pre_time = rospy.get_time()

                # Publish msg
                self.tdoa_msg.header = Header(frame_id=Recorder.FRAME_ID, stamp=rospy.Time.now())
                self.tdoa_msg.ch1_time = peak_time_ch1
                self.pub.publish(self.tdoa_msg)
                #rospy.loginfo("Publish %i samples of data", len(self.hydro_msg.data_ch1))

        def get_peak_time(self, time_start, index, fs):
            
            #delta_t = float(index1[0]) / self.rate # sec
            delta_t = float(index) / fs # sec
            epoch = int(delta_t * 1000000000)
            peak_time = time_start + epoch
            print "delta_t       : ", delta_t
            print "delta epoch   : ", epoch
            print "time final    : ", peak_time
            print "time final    : ", type(peak_time)
            return peak_time



        def onShutdown(self):
            rospy.loginfo("Recorder node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('recorder')
	node = Recorder()
        node.run()
        rospy.on_shutdown(node.onShutdown)
