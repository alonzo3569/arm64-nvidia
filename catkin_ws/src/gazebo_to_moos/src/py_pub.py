#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32 

if __name__ == '__main__':

    rospy.init_node('py_pub', anonymous = True) # Anonymous: If nodes have the same name, anonymous makes it possible to run both

    pub = rospy.Publisher("/CounterFromROS", Int32, queue_size=10) #Add buffer for subscriber in case they don't have time to process message

    rate = rospy.Rate(5) #two msg per second

    while not rospy.is_shutdown():
        msg = Int32() #Create an object from String class, in String class there is a private member called data
        msg.data = 333333333
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Node wass stopped")
