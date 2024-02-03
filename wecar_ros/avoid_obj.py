#! /usr/bin/env python
# -*- coding:utf-8 -*-

import math
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus

comeback = [-16.2082920074, 5.0082359314]

class avoid_object:
    def __init__(self):
        rospy.init_node('avoid_object', anonymous=False)
        rospy.Subscriber("/obj_info", PointCloud, self.avd_obj)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.receive_Ego_cb)
        self.avoid_path = rospy.Publisher("/avd_path", Float64, queue_size=10)

        self.selected_lane = 3
        

    def receive_Ego_cb(self, data):
        global car_position
        heading = data.heading
        car_x = data.position.x
        car_y = data.position.y
        # heading = heading*math.pi/180
        n = math.floor(heading / 360)
        Heading_180_to_360 = -(heading - n * 360)*math.pi/180 #라디안변환
        car_position = np.array([[car_x, car_y, Heading_180_to_360]]) 
        # print(Heading_180_to_360)
        
        # car_position = np.array([[car_x, car_y, heading]])

        # print(car_position)
        # print(type(car_position))

    def avd_obj(self, data):
        
        right_list = []
        left_list = []
        for j in data.points:

            obj_x = j.x
            obj_y = j.y

            if 5.25 < obj_y < 5.54:
                right_list.append([obj_x, obj_y])
            elif 4.9 < obj_y < 5.2:
                left_list.append([obj_x, obj_y])

        for i in car_position:
            if self.selected_lane == 3:
                for m in right_list:

                    obj_x = m[0]
                    obj_y = m[1]

                    dx = obj_x - i[0]
                    dy = obj_y - i[1]

                    dist = math.sqrt(dx*dx + dy*dy)
                    print(dist)

                    if dist <= 1.75:
                        self.selected_lane = 2

            if self.selected_lane == 2:
                for n in left_list:

                    obj_x = n[0]
                    obj_y = n[1]

                    dx = obj_x - i[0]
                    dy = obj_y - i[1]

                    dist = math.sqrt(dx*dx + dy*dy)

                    if dist <= 1.75:
                        self.selected_lane = 3

            car_x = i[0]
            car_y = i[1]
            
            dx = comeback[0] - car_x
            dy = comeback[1] - car_y

            dist = math.sqrt(dx*dx + dy*dy)

            if dist <= 2:
                self.selected_lane = 3


        self.avoid_path.publish(self.selected_lane)

if __name__ == '__main__':
    try:
        avoid_object()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



