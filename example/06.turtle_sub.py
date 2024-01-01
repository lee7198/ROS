#!/usr/bin/env python3
#-*-codingutf-8 -*-

import rospy
from turtlesim.msg import Pose

class Trutle_sub:
  def __init__(self):
    rospy.init_node("turtle_sub_node")
    rospy.Subscriber("turtle1/pose", Pose, self.callback)
    self.pose_msg = Pose()
    
  def callback(self, msg:Pose):
    print("x: %f, y: %f, theta: %f", msg.x, msg.y, msg.theta)

def main():
  try:
    turtle_sub = Trutle_sub()
    rospy.spin()  
  except rospy.ROSInterruptException:
    pass

if __name__ == "__main__":
  main()