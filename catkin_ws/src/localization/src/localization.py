#!/usr/bin/env python
'''
Author: Tony Hsiao                                                              
Date:2019/01/16                                                                
Last update: 2019/01/16                                                         
Locailization by gps and imu
'''
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geodesy.utm import UTMPoint, fromLatLong
#import message_filters
from tf.transformations import euler_from_quaternion
import math
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Pose
#import math
#from scipy.stats import norm
#import numpy as np

class Locailization(object):
    def __init__(self):
        
        # Params
        self.node_name = rospy.get_name()
        self.x = 0 # local coordinates
        self.y = 0

        # Onstartup
        self.lat_orig = rospy.get_param('~latitude', 0.0)  # (0,0) in moos
        self.long_orig = rospy.get_param('~longitude', 0.0)
        self.utm_orig = fromLatLong(self.lat_orig, self.long_orig)

        # Publisher
        self.pub_NavX = rospy.Publisher("NAV_X", Float64, queue_size=1);
        self.pub_NavY = rospy.Publisher("NAV_Y", Float64, queue_size=1);
        self.pub_NavLat = rospy.Publisher("NAV_LAT", Float64, queue_size=1);
        self.pub_NavLon = rospy.Publisher("NAV_LONG", Float64, queue_size=1);
        self.pub_NavHdg = rospy.Publisher("NAV_HEADING", Float64, queue_size=1);
        self.pub_NavSpeed = rospy.Publisher("NAV_SPEED_X", Float64, queue_size=1);

        #Subscriber
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.cb_imu)
        self.gps_sub = rospy.Subscriber("/fix", NavSatFix, self.cb_gps)
        #sub_imu = message_filters.Subscriber("/imu", Imu)
        #sub_gps = message_filters.Subscriber("/fix", NavSatFix)
        #ats = message_filters.ApproximateTimeSynchronizer([sub_imu, sub_gps], queue_size = 5, slop = 1.0)
        #ats.registerCallback(self.cb_gps_imu)

    def cb_gps(self, msg):
        utm_point = fromLatLong(msg.latitude, msg.longitude)
        self.x = utm_point.easting - self.utm_orig.easting
        self.y = utm_point.northing - self.utm_orig.northing
        self.pub_NavX.publish(self.x)
        self.pub_NavY.publish(self.y)
        self.pub_NavLat.publish(msg.latitude)
        self.pub_NavLon.publish(msg.longitude)
        self.pub_NavSpeed.publish(0.0)


    def cb_imu(self, msg):
        quaternion = (
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w)

        (roll,pitch,yaw) = euler_from_quaternion(quaternion)
        self.pub_NavHdg.publish(yaw*(180.0/math.pi))
        #print(round(roll*(180.0/math.pi),2))

    #def cb_gps_imu(self, msg_imu, msg_gps):
    #    rospy.loginfo("cb_gps")
    #    self.cb_gps(msg_gps)
    #    self.cb_imu(msg_imu)


    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('localization',anonymous=False)
    localization = Locailization()
    rospy.on_shutdown(localization.on_shutdown)
    rospy.spin()
