#! /usr/bin/env python
# -*- coding:utf-8 -*-

import math
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from morai_msgs.msg import EgoVehicleStatus

class identify_object:
    def __init__(self):
        rospy.init_node('identify_object', anonymous=False)
        rospy.Subscriber("/PC_MapData", PointCloud, self.identify_obj)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.receive_Ego_cb)
        self.object_info = rospy.Publisher("/obj_info", PointCloud, queue_size=10)

    def receive_Ego_cb(self, data):
        global car_position
        heading = data.heading
        car_x = data.position.x
        car_y = data.position.y
        # heading = heading*math.pi/180
        n = math.floor(heading / 360)
        Heading_180_to_360 = -(heading - n * 360)*math.pi/180 #라디안변환
        car_position = np.array([[car_x, car_y, Heading_180_to_360]]) 

    def identify_obj(self, data):
        header = data.header
        obj_info = PointCloud()
        obj_info.header = header

        for i in car_position:
            car_x = i[0]
            car_y = i[1]
            for j in data.points:
                obj_x = j.x
                obj_y = j.y

                if -19.5 < obj_x < car_x:
                    if 4 < obj_y < 5.9:
                        obj_info.points.append(Point32(obj_x, obj_y, 0))

        self.object_info.publish(obj_info)  

if __name__ == "__main__":
    try:
        identify_object()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass