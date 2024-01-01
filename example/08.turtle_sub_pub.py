#!/usr/bin/env python3
#-*-codingutf-8 -*-

# sub 기반 pub

import rospy
from turtlesim.msg import Pose, Color
from geometry_msgs.msg import Twist

class Trutle_sub:
  def __init__(self):
    rospy.init_node("turtle_sub_node")
    rospy.Subscriber("turtle1/pose", Pose, self.callback_pose)
    rospy.Subscriber("/turtlel/color_sensor", Color,self. callback_color)
    self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
    self.cmd_msg = Twist()
    self.pose_msg = Pose()
    self.color_msg = Color()
    self.rate = rospy.Rate(1)
    
  def callback_pose(self, msg:Pose):
    print("x: %f, y: %f, theta: %f", msg.x, msg.y, msg.theta)
  
  def callback_color(self, msg:Color):
    print("r: %f, g: %f, b: %f", msg.r, msg.g, msg.b)
    
def main():
  try:
    turtle_sub = Trutle_sub()
    rospy.spin()  
  except rospy.ROSInterruptException:
    pass

if __name__ == "__main__":
  main()