#! /usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import tf
import ros_numpy
import numpy as np
import math

from sensor_msgs.msg import PointCloud2, PointCloud
from morai_msgs.msg import EgoVehicleStatus

class LiDARParser:
    def __init__(self):
        rospy.init_node("lidar_convert_to_map", anonymous=False)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_cb)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_cb)
        self.pc2_pub = rospy.Publisher("/velodyne_map", PointCloud2, queue_size=1)
        
    def ego_status_cb(self, msg):
        self.ego_x, self.ego_y = msg.position.x, msg.position.y
        n = math.floor(msg.heading / 180)
        self.ego_heading_rad = -(msg.heading - n * 360)*math.pi/180
        
    def lidar_cb(self, msg):
        array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        
        rotate_x = array[:, 0]*math.cos(self.ego_heading_rad)+array[:, 1]*math.sin(self.ego_heading_rad)
        rotate_y = -array[:, 0]*math.sin(self.ego_heading_rad)+array[:, 1]*math.cos(self.ego_heading_rad)
        map_x = rotate_x + self.ego_x 
        map_y = rotate_y + self.ego_y
        
        pc2_array = np.zeros(len(array), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
        ])
        pc2_array['x'] = map_x
        pc2_array['y'] = map_y
        pc2_array['z'] = array[:, 2]
        pc2_array['intensity'] = 255

        pc_msg = ros_numpy.msgify(PointCloud2, pc2_array, stamp=msg.header.stamp, frame_id="map")
        
        self.pc2_pub.publish(pc_msg)
        
if __name__ == "__main__":
    lp = LiDARParser()
    rospy.spin()
        