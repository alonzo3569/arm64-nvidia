#!/usr/bin/env python
'''
Title  : Test the effect of kalman filter using Turtlesim (Constant velocity model)
Author : logan zhang
'''
import rospy
import numpy as np
import math
from std_msgs.msg import Float64
from turtlesim.msg import Pose
from ntu_msgs.msg import Kalman
import matplotlib.pyplot as plt

class KalmanFilter(object):

    # Config
    GPS_STD = 0.1
    VEL_STD = 0.01
    PLOT_TIME = 20

    def __init__(self):

        # Params
        self.node_name = rospy.get_name()
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_vx = 0.0
        self.gt_vy = 0.0
        self.msg = Kalman()
        self.start = rospy.get_time()
        self.plot = False
        self.gt_x_list = []
        self.gt_y_list = []

        # Kalman Params
        self.pre_time = rospy.get_time()
        self.X = np.zeros(4)[:,np.newaxis]
        self.Z = np.array([])
        self.H = np.eye(4)
        self.Q = np.eye(4)
        self.I = np.eye(4)

        self.P = np.diag([100.0, 100.0, 1.0, 1.0])
        self.R = np.diag([100.0, 100.0, 1.0, 1.0])


        # Publisher
        #self.pub = rospy.Publisher("kalman", Kalman, queue_size=1);

        #Subscriber
        self.pose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.pose_cb)

    def pose_cb(self, msg):

        # Plot after PLOT_TIME sec
        if rospy.get_time() - self.start > KalmanFilter.PLOT_TIME:
            if self.plot == False:
                fig, ax = plt.subplots(1, figsize=(30,30))
                ax.scatter(self.msg.gps_x, self.msg.gps_y, 15, label='gps') # 15: size of scatter point
                ax.scatter(self.gt_x_list, self.gt_y_list, 15, label='ground truth')
                ax.scatter(self.msg.kalman_x, self.msg.kalman_y, 15, label='kalman filter(gps+imu)')
                ax.legend(loc='upper right')
                ax.set_title('ground truth & sensor data & filtered')
                plt.show()
                self.plot = True
            return

        # Sub ground truth msg
        self.gt_x = msg.x
        self.gt_y = msg.y
        self.gt_vx = msg.linear_velocity * math.cos(msg.theta)
        self.gt_vy = msg.linear_velocity * math.sin(msg.theta)
        self.gt_x_list.append(self.gt_x)
        self.gt_y_list.append(self.gt_y)

        # Simulate sensor data
        self.gps_x = self.add_gaussian_noise(self.gt_x, KalmanFilter.GPS_STD)
        self.gps_y = self.add_gaussian_noise(self.gt_y, KalmanFilter.GPS_STD)
        self.sensor_vx = self.add_gaussian_noise(self.gt_vx, KalmanFilter.VEL_STD)
        self.sensor_vy = self.add_gaussian_noise(self.gt_vy, KalmanFilter.VEL_STD)

        # Write ros msg
        self.msg.gps_x.append(self.gps_x)
        self.msg.gps_y.append(self.gps_y)

        # Combine sim data into Z(measurement) matrix
        self.Z = np.array([self.gps_x, self.gps_y, self.sensor_vx, self.sensor_vy])

        # Calculate dt
        cur_time = rospy.get_time()
        dt = cur_time - self.pre_time
        self.pre_time = cur_time
        print"Time difference dt : ", dt

        # Kalman filter
        self.kalman_filter(dt)

        # Publish msg
        #if len(self.msg.gps_x) > 100 :
        #    self.pub.publish(self.msg)
        #    # Clear msg
        #    del self.msg.gps_x[:]
        #    del self.msg.gps_y[:]
        #    del self.msg.kalman_x[:]
        #    del self.msg.kalman_y[:]


    def add_gaussian_noise(self, gt_data, stdev):
        noise_data = gt_data + stdev * np.random.randn()
        return noise_data


    def kalman_filter(self, dt):

        A = np.matrix([[1.0, 0.0, dt, 0.0],
                       [0.0, 1.0, 0.0, dt],
                       [0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]])


        # Time Update (Prediction)
        # ========================
        # Project the state ahead
        self.X = A*self.X

        # Project the error covariance ahead
        self.P = A*self.P*A.T + self.Q


        # Measurement Update (Correction)
        # ===============================
        # Compute the Kalman Gain
        S = self.H*self.P*self.H.T + self.R
        K = (self.P*self.H.T) * np.linalg.pinv(S)


        # Update the estimate via z
        self.Z = self.Z[:,np.newaxis]
        y = self.Z - (self.H*self.X)        # Innovation or Residual
        self.X = self.X + (K*y)

        # Update the error covariance
        self.P = (self.I - (K*self.H))*self.P

        # Save states (for Plotting)
        self.msg.kalman_x.append(self.X[0])
        self.msg.kalman_y.append(self.X[1])

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('kalman',anonymous=False)
    kalman = KalmanFilter()
    rospy.on_shutdown(kalman.on_shutdown)
    rospy.spin()
