#!/usr/bin/env python
'''
Title : Locailization by gps and imu (kalman filter)
Author: logan zhang, tony hsiao                                                            
'''
import rospy
import math
import message_filters
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from geodesy.utm import UTMPoint, fromLatLong
from tf.transformations import euler_from_quaternion
from ntu_msgs.msg import RobotStatus
from std_msgs.msg import Header

class Locailization(object):

    # Static Params
    FRAME_ID =  "base_link"

    def __init__(self):
        
        # Params
        self.node_name = rospy.get_name()
        self.x = 0.0 # local coordinates
        self.y = 0.0
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.status = RobotStatus()

        # Initialize Kalman
        self.pre_time = rospy.get_time()
        self.X = np.zeros(6)[:,np.newaxis]
        self.Z = np.array([])
        self.H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        self.Q = np.eye(6)
        self.I = np.eye(6)

        self.P = np.diag([100.0, 100.0, 10.0, 10.0, 1.0, 1.0])
        self.R = np.diag([100.0, 100.0, 1.0, 1.0]) # Should subscribe from sensor covariance


        # Onstartup
        self.lat_orig = rospy.get_param('~latitude', 0.0)  # (0,0) in moos
        self.long_orig = rospy.get_param('~longitude', 0.0)
        self.utm_orig = fromLatLong(self.lat_orig, self.long_orig)

        # Publisher
        self.pub = rospy.Publisher("robot_status", RobotStatus, queue_size=1);

        #Subscriber
        imu_sub = message_filters.Subscriber("/imu", Imu)
        gps_sub = message_filters.Subscriber("/fix", NavSatFix)
        ats = message_filters.ApproximateTimeSynchronizer([imu_sub, gps_sub], queue_size = 5, slop = 0.01) # slope : error between 2 cb 
        ats.registerCallback(self.cb_imu_gps)

    def cb_gps(self, msg):
        utm_point = fromLatLong(msg.latitude, msg.longitude)
        self.x = utm_point.easting - self.utm_orig.easting
        self.y = utm_point.northing - self.utm_orig.northing

    def cb_imu(self, msg):
        quaternion = (
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w)

        (roll,pitch,yaw) = euler_from_quaternion(quaternion)

        # From rad to deg and change to moos heading
        yaw = math.degrees(yaw)
        yaw = self.toMoosAngle(yaw)
        self.status.heading = yaw
        print"MOOS heading : ", yaw

        # Acc in heading direction to local coordinate ax, ay
        self.acc_x = 0 * (- msg.linear_acceleration.x) * math.sin(math.radians(yaw))
        self.acc_y = 0 * (- msg.linear_acceleration.x) * math.cos(math.radians(yaw))

    def cb_imu_gps(self, msg_imu, msg_gps):

        # Calcualate dt between each loop
        cur_time = rospy.get_time()
        dt = cur_time - self.pre_time
        print"dt : ", dt
        self.pre_time = cur_time

        self.cb_imu(msg_imu)
        self.cb_gps(msg_gps)

        # Measuremant matrix
        self.Z = np.array([ self.x, self.y, self.acc_x, self.acc_y])

        self.kalman_filter(dt)

        # Publish robot status
        self.status.header = Header(frame_id=Locailization.FRAME_ID, stamp=rospy.Time.now())
        self.pub.publish(self.status)



    def kalman_filter(self, dt):

        A = np.matrix([[1.0, 0.0, dt, 0.0, 1/2.0*dt**2, 0.0],
                      [0.0, 1.0, 0.0, dt, 0.0, 1/2.0*dt**2],
                      [0.0, 0.0, 1.0, 0.0, dt, 0.0],
                      [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                      [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

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
        self.status.local_x = self.X[0]
        self.status.local_y = self.X[1]

    def toMoosAngle(self, theta):
        if theta < 0.0:
            theta = -theta
        else:
            theta = -theta + 360.0
        return theta


    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('localization',anonymous=False)
    localization = Locailization()
    rospy.on_shutdown(localization.on_shutdown)
    rospy.spin()
