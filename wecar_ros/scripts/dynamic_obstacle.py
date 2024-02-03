#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from morai_msgs.msg import GetTrafficLightStatus, EgoVehicleStatus
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path

class DynamicStop:
    def __init__(self):
        rospy.init_node("dynamic_stop", anonymous=False)

        # subscriber
        rospy.Subscriber("/laser2pcd_map", PointCloud, self.callback)
        rospy.Subscriber("/local_path", Path, self.local_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_cb)
        
        # publisher
        self.vel_pub = rospy.Publisher("/dynamic_vel", Float64, queue_size=10)

        #초기값 설정
        self.dynamic_vel = 999
        self.local_path_list = []

    def callback(self, msg):
        self.pcd_list = []
        for i in msg.points:
            pcd = []
            pcd.append(i.x)
            pcd.append(i.y)

            self.pcd_list.append(pcd)

        '''알고리즘 부분'''
        #Local path 에 장애물이 존재하는지 검사
        obstacle_list = []
        for i in self.pcd_list:
            for j in self.local_path_list:
                dx = (i[0]-j[0])
                dy = (i[1]-j[1])
                dst = math.sqrt((dx*dx)+(dy*dy))
                if dst < 0.8:
                    # 장애물 발견
                    obstacle_list.append(i)
                else:
                    self.dynamic_vel = 999

        for i in obstacle_list:
            dx = (i[0]-self.ego_x)
            dy = (i[1]-self.ego_y)
            dst = math.sqrt((dx*dx)+(dy*dy))
            if dst < 1.5:
                # print("서라")
                self.dynamic_vel = 0
            
        
        self.vel_pub.publish(self.dynamic_vel)


    def local_cb(self, msg):
        # Local path 를 받아와서 processing
        self.local_path_list = []
        for i in msg.poses:
            path = []
            path.append(i.pose.position.x)
            path.append(i.pose.position.y)
            # path.append(i.pose.position.z)

            self.local_path_list.append(path)
        # print(self.path_list)

    def ego_cb(self, msg):
        self.ego_x = msg.position.x
        self.ego_y = msg.position.y


if __name__ == "__main__":
    ts = DynamicStop()
    rospy.spin()