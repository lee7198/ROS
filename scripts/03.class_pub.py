#!/usr/bin/env python3
#-*-codingutf-8 -*-

import rospy
from std_msgs.msg import Int32

class Class_pub:
    def __init__(self):
        # name
        rospy.init_node('node_pub')
        # role
        self.pub = rospy.Publisher('/counter', Int32, queue_size=1)
        # type
        self.int_msg = Int32()
        # rate (Hz)
        self.rate = rospy.Rate(11)
    
    def func(self):
        num = 0
        while not rospy.is_shutdown():
            num = num + 1
            self.int_msg.data = num
            # sned msg
            self.pub.publish(self.int_msg)
            # set rate
            self.rate.sleep()

def main():
    try :
        class_pub = Class_pub()
        class_pub.func()
    except rospy.ROSInterruptException: 
        pass

# main fn
if __name__ == "__main__":
    main()