#!/usr/bin/env python3
#-*-codingutf-8 -*-

# pub 기반 sub

import rospy
from turtlesim.msg import Pose, Color
from geometry_msgs.msg import Twist

class Trutle_sub:
  def __init__(self):
    rospy.init_node("turtle_sub_node")
    rospy.Subscriber("turtle1/pose", Pose, self.callback_pose)
    # rospy.Subscriber("/turtlel/color_sensor", Color,self. callback_color)
    self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
    self.cmd_msg = Twist()
    self.pose_msg = Pose()
    self.color_msg = Color()
    self.rate = rospy.Rate(10)
  
  def func(self):
    self.cmd_msg.linear.x = 1
    
    if self.color_msg.r > 100: 
      if 1< self.pose_msg.x < 10 and 1< self.pose_msg.y < 10:
        self.cmd_msg.linear.x = 5
        self.cmd_msg.angular.z = 0
      else:
        self.cmd_msg.linear.x = 0.7
        self.cmd_msg.angular.z = 1
    else:
      print("already pass")
    
    # if self.pose_msg.x > 8:
    #   self.cmd_msg.linear.x = 0
    #   if self.color_msg.r > 100:
    #     self.cmd_msg.angular.z = 1
    #   else:
    #     self.cmd_msg.angular.z = 0
        
    self.pub.publish(self.cmd_msg)
    # 기본 64Hz와 프로그램내 설정한 rate의 차이 때문에 통신 지연 있을 수 있음
    self.rate.sleep()
    
  def callback_pose(self, msg:Pose):
    self.pose_msg = msg
    print(msg.x, msg.y, msg.theta)
    
  
  def callback_color(self, msg:Color):
    self.color_msg = msg
    print(msg.r, msg.g, msg.b)
    
def main():
  try:
    turtle_sub = Trutle_sub()
    while not rospy.is_shutdown():
      turtle_sub.func()
  except rospy.ROSInterruptException:
    pass

if __name__ == "__main__":
  main()