#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan,PointCloud
from math import cos,sin,pi
from geometry_msgs.msg import Point32

class lidarParser :

    def __init__(self):
        rospy.init_node('lidar_parser', anonymous=True)

        # subscriber
        rospy.Subscriber("/lidar2D", LaserScan, self.laser_callback)

        # publihser
        self.angle_pub = rospy.Publisher('angle',Float64, queue_size=1)
        self.x_pub = rospy.Publisher('x',Float64, queue_size=1)
        self.y_pub = rospy.Publisher('y',Float64, queue_size=1)

        rospy.spin()


    def laser_callback(self,msg):
        pcd=PointCloud()
        motor_msg=Float64()
        pcd.header.frame_id=msg.header.frame_id
        angle=0
        degree=0
        print(pcd)

        for r in msg.ranges :

            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)

            x=r*cos(angle)
            y=r*sin(angle)
            angle=angle+(1.0/180*pi) #rad
            #print("degree : ",round(angle*180/pi)) #degree
            #print(angle)
            #rad = angle * (pi / 180.0)
            nx = round(cos(angle)*x - sin(angle)*y)
            ny = round(sin(angle)*x + cos(angle)*y)
            print("angle : {:.2f} nx : {:.2f} ny : {:.2f}".format(angle, nx, ny))         
            #print("angle : {:.2f} x : {:.2f} y : {:.2f}".format(angle, x, y))         
            #print("angle : {:.2f} x : {:.2f} y : {:.2f}".format(angle, tmp_point.x, tmp_point.x))
            #print("x : {:.2f} nx : {:.2f}".format(x, nx))         
            #print("angle : {:.2f} rad : {:.2f}".format(angle, rad))         
            #print("rad : {:2f} angle : {:2f}".format(rad,angles))
            



            if r<12  :
                pcd.points.append(tmp_point)
                #print(tmp_point)


if __name__ == '__main__':
    try:
        test=lidarParser()
    except rospy.ROSInterruptException:
        pass