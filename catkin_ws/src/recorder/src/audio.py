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
            # Device id : 0(xps), 6(focusrite)
            p = pyaudio.PyAudio()
            self.stream = p.open(format=self.format,
                            channels=self.channels,
                            rate=self.rate,
                            input=True,
                            input_device_index=4,
                            frames_per_buffer=self.chunk)

            # Initialize msg
            self.hydro_msg = HydrophoneData()

            # Publisher
	    self.pub = rospy.Publisher("/hydrophone_data", HydrophoneData, queue_size=10)


        def run(self):
            while not rospy.is_shutdown():
                # Read data stream, if chunk size is too small, data will be overwrite by new data
                # type(data) : str
                data = self.stream.read(self.chunk, exception_on_overflow = False)

                # Convert string to numpy array
                # data array element type : int32
                data_array = np.fromstring(data, dtype='int32')

                # Deinterleave, select 1 channel
                # ch_1, ch_2 type : float64
                # size of ch_1, ch_2 : self.chunk=19200, shape : (19200,)
                ch_1 = data_array[0::self.channels] / 2.0**31 
                ch_2 = data_array[1::self.channels] / 2.0**31
                #print "ch_1 shape", ch_1.shape

                # Append ch data to list
                # shape of self.hydro_msg.data_ch1 : (19200,)
                # len(data_ch1) = (19200~38400~,)
                self.hydro_msg.data_ch1.extend(ch_1.tolist())
                self.hydro_msg.data_ch2.extend(ch_2.tolist())
                #print "shape: ", np.array(self.hydro_msg.data_ch1).shape
                #print "len: ", len(self.hydro_msg.data_ch1)

                # Publish msg
                if len(self.hydro_msg.data_ch1) >= self.rate * self.msg_length:
                    self.hydro_msg.header = Header(frame_id=Recorder.FRAME_ID, stamp=rospy.Time.now()
)
                    self.hydro_msg.length = len(self.hydro_msg.data_ch1)
                    self.hydro_msg.fs = self.rate
                    self.pub.publish(self.hydro_msg)
                    rospy.loginfo("Publish %i samples of data", len(self.hydro_msg.data_ch1))

                    # Clear after publish
                    del self.hydro_msg.data_ch1[:]
                    del self.hydro_msg.data_ch2[:]

        def onShutdown(self):
            rospy.loginfo("Recorder node Shutdown.")


if __name__ == '__main__':
	rospy.init_node('recorder')
	node = Recorder()
        node.run()
        rospy.on_shutdown(node.onShutdown)
