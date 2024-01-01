#!/usr/bin/env python3
#-*-codingutf-8 -*-

import rospy
from std_msgs.msg import Int32

class Class_sub:
    def __init__(self):
        # name
        rospy.init_node('node_sub')
        # role
        rospy.Subscriber('/counter', Int32, callback=self.CB)
    
    # sub's callback fn
    def CB(msg):
        print(msg)

def main():
    try :
        class_sub = Class_sub()
        # run
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass

# main fn
if __name__ == "__main__":
    main()