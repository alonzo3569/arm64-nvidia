#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData
from std_msgs.msg import Header


class PLOT:

    # Static params
    FRAME_ID = "base_link"
    PLOT_FFT_TIME = 5

    def __init__(self):
        # OnstartUp
        self.chunk = rospy.get_param('~fft_chunk', 1024) # 1024
        self.fs = rospy.get_param('~rate', 192000) # 192000

        # Initialize fig
        self.fig, self.ax = plt.subplots(figsize=(10,3))
        self.plot_fft_ch1 = np.zeros((3,3))
        self.im = self.ax.imshow(self.plot_fft_ch1, cmap=plt.cm.jet, origin='lower', vmin=-100, vmax=0, aspect='auto')
        self.fig.colorbar(self.im, ax=self.ax)
        #self.ax.set_label_position()
        self.ax.axis('auto')
        self.ax.set_ylim([0, 20])
        self.ax.set_xlabel("Time(s)")
        self.ax.set_ylabel("Frequency(kHz)")

        # Initialize params
        self.first_msg = True
        self.delta_f = 0.0
        self.delta_t = 0.0
        self.counter = 0
        self.msg_number = 0
        self.time_pre = 0.0
        self.fft_result_len = self.chunk//2 + 1
        self.animation_start = rospy.get_time()


        # Subscriber
        rospy.Subscriber("/fft", HydrophoneFFTData, self.fft_cb)



    def fft_cb(self, msg):
        # Calculate msg recieved
        self.msg_number = self.msg_number + 1
        print(f'Recieved number {self.msg_number} message')
        self.counter = self.counter + 1
        time_cur = rospy.get_time()
        if time_cur - self.time_pre >= 1.0:
            print(f'Recieved {self.counter} msgs in 1 sec')
            self.counter = 0
            self.time_pre = time_cur

        # Initialize plot frame base on msg
        if self.first_msg == True:
            self.delta_f = msg.delta_f
            self.delta_t = msg.delta_t
            self.plot_fft_ch1 = np.zeros((self.fft_result_len,int(PLOT.PLOT_FFT_TIME//self.delta_t)))
            #print(f'shape of plot array : {self.plot_fft_ch1.shape}') # (513, 9375)
            self.first_msg = False

        # Subscribe fft data and change into 2 dimension array
        ch1 = np.array(msg.fft_ch1).reshape(int(len(msg.fft_ch1)/self.fft_result_len),self.fft_result_len).T # +211 #(513, 1882)

        # Remove previous data and append new data
        self.plot_fft_ch1 = self.plot_fft_ch1[:,ch1.shape[1]:] # (513, 7493)
        self.plot_fft_ch1 = np.concatenate((self.plot_fft_ch1,ch1),axis=1) # (513, 9375)



    def animation_frame(self,i):
        self.im.set_array(self.plot_fft_ch1)
        time_end = int(rospy.get_time() - self.animation_start)
        time_start = time_end - PLOT.PLOT_FFT_TIME
        self.im.set_extent([time_start, time_end, 0, self.fs//2//1000])
        #self.im.set_extent([0,5,0,96])
        return [self.im]


        
    def onShutdown(self):
        rospy.loginfo("FFT node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('plot')
    node = PLOT()
    animation = FuncAnimation(node.fig, node.animation_frame, interval=500, blit=True)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
