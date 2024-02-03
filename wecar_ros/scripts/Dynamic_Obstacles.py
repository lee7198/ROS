#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import LaserScan, PointCloud
from math import cos,sin,pi
from geometry_msgs.msg import Point32

class obstaclestop:
    def __init__(self):  
        rospy.init_node("Dynamic_Obstacles", anonymous=True)
        rospy.Subscriber("/lidar2D", LaserScan, self.laser_callback)
        rospy.Subscriber("/current_waypoint", Int32, self.waypoint_cb)
       
        self.vel_pub = rospy.Publisher("obstacles_vel", Float64, queue_size=10)
        #self.pcd_pub = rospy.Publisher('laser2pcd',PointCloud, queue_size=1)
        


    def laser_callback(self,msg):
        pcd=PointCloud()
        motor_msg=Float64()
        pcd.header.frame_id=msg.header.frame_id  ## 장애물 상태
        angle=0

        obstacles_vel = 999 ## 장애물 속도 초기화

        vehicle_status=[self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi,self.status_msg.velocity.x/3.6]
        print(vehicle_status)

        for r in msg.ranges :
            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)
            angle=angle+(1.0/180*pi)

            #print(tmp_point.x,tmp_point.y)
            #print(tmp_point)
            if r<5 :
                pcd.points.append(tmp_point)
               # print(tmp_point)
                obstacles_vel = 0

                

        self.vel_pub.publish(obstacles_vel)         
        #self.pcd_pub.publish(pcd)
        def waypoint_cb(self, msg):
            self.current_waypoint = msg.data

  

   

if __name__ == "__main__":
    os = obstaclestop()
    rospy.spin()