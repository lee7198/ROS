#!/usr/bin/env python3
#-*-codingutf-8 -*-

import rospy
# msg type
from std_msgs.msg import Int32

# setting node name
rospy.init_node('edu_pub_node')
# node role
pub = rospy.Publisher("/counter", Int32, queue_size=1)
int_msg = Int32()
# limit ros rate (Hz)
rospy.Rate(10)

num = 0
while not rospy.is_shutdown():
    num = num + 1
    int_msg.data = num
    # sned msg
    pub.publish(int_msg)