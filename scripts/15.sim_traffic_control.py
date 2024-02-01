#!/usr/bin/env python3
# -*-codingutf-8 -*-

# sub 기반 pub

import rospy
from morai_mggs.msg import GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np


class Traffic_control:
    def __init__(self):
        rospy.init_node("Traffic_control_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.traffic_CB)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher(
            "/commands/servo/position", Float64, queue_size=1
        )
        self.speed_msg = Float64()
        self.steer_msg = Float64()

    def traffic_CB(self, msg: CompressedImage):
        cv2.waitKey(1)


def main():
    try:
        Traffic_control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
