#!/usr/bin/env python3
#-*-codingutf-8 -*-

import rospy
# msg type
from std_msgs.msg import Int32

# sub's callback fn
def CB(msg):
    print(msg)

# setting node name
rospy.init_node('edu_sub_node')
# node role
rospy.Subscriber("counter", Int32, callback=CB)
rospy.spin()