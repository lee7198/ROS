#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from morai_msgs.msg import GetTrafficLightStatus, EgoVehicleStatus
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Path
import time

class DynamicStop:
    def __init__(self):
        rospy.init_node("dynamic_stop", anonymous=False)

        # subscriber
        rospy.Subscriber("/laser2pcd_map", PointCloud, self.callback)
        rospy.Subscriber("/local_path", Path, self.local_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_cb)
        
        # publisher
        self.vel_pub = rospy.Publisher("/dynamic_vel", Float64, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/obstacle_mean", PointCloud, queue_size=1)

        #초기값 설정
        self.dynamic_vel = 999
        self.local_path_list = []
        self.tic_tok = 0 
        self.static_mission = False

    def callback(self, msg):
        obstacle_mean = PointCloud()
        obstacle_mean.header.frame_id = "map"

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
                    obstacle_list.append(i)
                else:
                    self.dynamic_vel = 999

        # if len(obstacle_list) > 0:
        #     obstacle_np = np.array(obstacle_list)
        #     obj_x_mean = np.mean(obstacle_np.T[0])
        #     obj_y_mean = np.mean(obstacle_np.T[1])
        #     obstacle_mean.points.append(Point32(obj_x_mean, obj_y_mean, 0))

        #     pre_obj_x = obj_x_mean
        #     pre_obj_y = obj_y_mean
        
        # print("pre : ", pre_obj_x, pre_obj_y)
        # print("obj : ", obj_x_mean, obj_y_mean)

        # self.obstacle_pub.publish(obstacle_mean)

        for i in obstacle_list:
            dx = (i[0]-self.ego_x)
            dy = (i[1]-self.ego_y)
            dst = math.sqrt((dx*dx)+(dy*dy))
            if dst < 1.5:
                self.dynamic_vel = 0

        # 정적인지 동적인지 구분
        if self.dynamic_vel == 0:
            self.tic_tok += 1
        else:
            self.tic_tok = 0

        # 틱톡 갯수 프린팅
        # print(self.tic_tok)

        if self.tic_tok > 50:
            self.static_mission = True
            print("정적 장애물입니다!")

        # 정적 장애물 회피 알고리즘
        '''
        if self.static_mission == True:
            여기에 회피 알고리즘 짜면됨
        '''

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