#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Int16, Float32MultiArray, Int32
from geometry_msgs.msg import PoseStamped, Point32
from morai_msgs.msg import (
    EgoVehicleStatus,
    ObjectStatusList,
    CtrlCmd,
    GetTrafficLightStatus,
    SetTrafficLight,
)
from lib.utils import (
    pathReader,
    findLocalPath,
    purePursuit,
    cruiseControl,
    vaildObject,
    pidController,
    velocityPlanning,
    latticePlanner,
)
import tf
from math import cos, sin, sqrt, pow, atan2, pi


class wecar_planner:
    def __init__(self):
        rospy.init_node("wecar_planner", anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        for item in arg:
            print(f"hello {item}")
        self.path_name = arg[0]

        # publisher
        global_path_pub = rospy.Publisher(
            "/global_path", Path, queue_size=1
        )  ## global_path publisher
        local_path_pub = rospy.Publisher(
            "/local_path", Path, queue_size=1
        )  ## local_path publisher
        self.motor_pub = rospy.Publisher("commands/motor/speed", Float64, queue_size=1)
        self.servo_pub = rospy.Publisher(
            "commands/servo/position", Float64, queue_size=1
        )
        self.wayp_pub = rospy.Publisher("/current_waypoint", Int32, queue_size=1)
        # self.status_pub = rospy.Publisher('status_msg', Int64, queue_size=1)

        ########################  lattice  ########################
        for i in range(1, 8):
            globals()["lattice_path_{}_pub".format(i)] = rospy.Publisher(
                "lattice_path_{}".format(i), Path, queue_size=1
            )
        ########################  lattice  ########################

        # subscriber
        rospy.Subscriber(
            "/Ego_topic", EgoVehicleStatus, self.statusCB
        )  ## Vehicl Status Subscriber

        # mission velocity subscriber
        rospy.Subscriber("/traffic_vel", Float64, self.traffic_set)
        rospy.Subscriber("/dynamic_vel", Float64, self.dynamic_set)

        # lidar 좌표 subscriber
        rospy.Subscriber("angle", Float64, self.angle_set)
        rospy.Subscriber("x", Float64, self.x_set)
        rospy.Subscriber("y", Float64, self.y_set)

        # mission dynamic velocity subscriber
        # rospy.Subscriber("obstacles_vel", Float64, self.obstacles_set)

        # rospy.Subscriber("/Object_topic", ObjectStatusList, self.objectInfoCB) ## Object information Subscriber

        # def
        self.is_status = False  ## 차량 상태 점검
        self.is_obj = False  ## 장애물 상태 점검
        # self.steering_angle_to_servo_offset=0.5304 ## servo moter offset
        self.steering_angle_to_servo_offset = 0.5000  ## servo moter offset
        # self.rpm_gain = 4616

        # 속도 초기화
        self.traffic_vel = 999  ## 신호등
        self.dynamic_vel = 999  ## 동적장애물
        # self.obstacles_vel = 999 ## 동적장애물

        self.rpm_gain = 1000
        self.motor_msg = Float64()
        self.servo_msg = Float64()

        # class
        path_reader = pathReader("wecar_ros")  ## 경로 파일의 위치
        pure_pursuit = purePursuit()  ## purePursuit import
        self.cc = cruiseControl(
            0.5, 1
        )  ## cruiseControl import (object_vel_gain, object_dis_gain)
        self.vo = vaildObject()  ## 장애물 유무 확인 ()
        pid = pidController()  ## pidController import
        # read path
        self.global_path = path_reader.read_txt(
            self.path_name + ".txt"
        )  ## 출력할 경로의 이름

        vel_planner = velocityPlanning(10, 0.15)  ## 속도 계획
        # vel_planner=velocityPlanning(8, 0.15) ## 속도 계획
        vel_profile = vel_planner.curveBasedVelocity(self.global_path, 30)

        # 값 초기화
        self.is_obj = True

        # time var
        count = 0
        rate = rospy.Rate(30)  # 30hz
        # lattice_current_lane=3
        lattice_current_lane = 4

        while not rospy.is_shutdown():
            # print(self.is_status , self.is_obj)
            # print(self.is_status)

            if (
                self.is_status == True and self.is_obj == True
            ):  ## 차량의 상태, 장애물 상태 점검
                ## global_path와 차량의 status_msg를 이용해 현재 waypoint와 local_path를 생성
                local_path, self.current_waypoint = findLocalPath(
                    self.global_path, self.status_msg
                )
                self.wayp_pub.publish(self.current_waypoint)

                # print(len(local_path.poses))
                ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)
                # self.vo.get_object(self.object_num,self.object_info[0],self.object_info[1],self.object_info[2],self.object_info[3])
                # global_obj,local_obj=self.vo.calc_vaild_obj([self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi])
                # print(object_num)
                global_obj = []
                local_obj = []
                ########################  lattice  ########################
                vehicle_status = [
                    self.status_msg.position.x,
                    self.status_msg.position.y,
                    (self.status_msg.heading) / 180 * pi,
                    self.status_msg.velocity.x / 3.6,
                ]
                lattice_path, selected_lane = latticePlanner(
                    local_path, global_obj, vehicle_status, lattice_current_lane
                )
                lattice_current_lane = selected_lane

                if selected_lane != -1:
                    local_path = lattice_path[selected_lane]
                    # print("local path len :", str(len(local_path.poses)))

                if len(lattice_path) == 7:
                    for i in range(1, 8):
                        globals()["lattice_path_{}_pub".format(i)].publish(
                            lattice_path[i - 1]
                        )
                ########################  lattice  ########################

                self.cc.checkObject(local_path, global_obj, local_obj)

                pure_pursuit.getPath(
                    local_path
                )  ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(
                    self.status_msg
                )  ## pure_pursuit 알고리즘에 차량의 status 적용

                self.steering = pure_pursuit.steering_angle()

                self.cc_vel = self.cc.acc(
                    local_obj,
                    self.status_msg.velocity.x,
                    vel_profile[self.current_waypoint],
                    self.status_msg,
                )  ## advanced cruise control 적용한 속도 계획

                # 속도 Filtering
                spd_list = []
                spd_list.append(self.cc_vel)  # wecar planner 계획한 현재 속도
                spd_list.append(self.traffic_vel)
                spd_list.append(self.dynamic_vel)
                # spd_list.append(self.obstacles_vel)

                # 속도 리스트 중에서 "최솟값"
                min_vel = min(spd_list)

                # print("waypoint : {:0f}".format(self.current_waypoint))
                print(
                    "min_vel {:.2f} / cc_vel : {:.2f} traffic_vel : {:.2f} dynamic_vel : {:.2f}".format(
                        min_vel, self.cc_vel, self.traffic_vel, self.dynamic_vel
                    )
                )
                # print("min_vel {:.2f} / cc_vel : {:.2f} traffic_vel : {:.2f} obstacles_vel : {:.2f}".format(min_vel, self.cc_vel, self.traffic_vel, self.obstacles_vel))

                # self.servo_msg = self.steering*0.021 + self.steering_angle_to_servo_offset
                self.servo_msg = (
                    self.steering * 0.0071 + self.steering_angle_to_servo_offset
                )
                # self.motor_msg = min_vel *self.rpm_gain /3.6
                self.motor_msg = min_vel * self.rpm_gain / 2

                local_path_pub.publish(local_path)  ## Local Path 출력

                self.servo_pub.publish(self.servo_msg)
                self.motor_pub.publish(self.motor_msg)
                self.print_info()

            if count == 300:  ## global path 출력
                global_path_pub.publish(self.global_path)
                count = 0
            count += 1

            rate.sleep()

    def print_info(self):

        # os.system('clear')
        # print('--------------------status-------------------------')
        # print('position :{0} ,{1}, {2}'.format(self.status_msg.position.x,self.status_msg.position.y,self.status_msg.position.z))
        # print('velocity :{} km/h'.format(self.status_msg.velocity.x))
        # print('heading :{} deg'.format(self.status_msg.heading))

        # # print('--------------------object-------------------------')
        # print('object num :{}'.format(self.object_num))
        # for i in range(0,self.object_num) :
        # print('{0} : type = {1}, x = {2}, y = {3}, z = {4} '.format(i,self.object_info[0],self.object_info[1],self.object_info[2],self.object_info[3]))

        # print('--------------------controller-------------------------')
        # print('target vel_planning :{} km/h'.format(self.cc_vel))
        # print('target steering_angle :{} deg'.format(self.steering))

        # print('--------------------localization-------------------------')
        # print('all waypoint size: {} '.format(len(self.global_path.poses)))
        # print('current waypoint : {} '.format(self.current_waypoint))
        pass

    def statusCB(self, data):  ## Vehicl Status Subscriber
        self.status_msg = data
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (
                self.status_msg.position.x,
                self.status_msg.position.y,
                self.status_msg.position.z,
            ),
            tf.transformations.quaternion_from_euler(
                0, 0, (self.status_msg.heading) / 180 * pi
            ),
            rospy.Time.now(),
            "gps",
            "map",
        )
        self.is_status = True
        # print('x : {:.2f}, y : {:.2f}'.format(self.status_msg.position.x, self.status_msg.position.y))

    def traffic_set(self, msg):
        self.traffic_vel = msg.data
        # print(self.traffic_vel))

    def x_set(self, msg):
        self.x = msg.data
        # print("x : {:.2f}".format(self.x))

    def y_set(self, msg):
        self.y = msg.data
        # print("y : {:.2f}".format(self.y))

    def angle_set(self, msg):
        self.angle = msg.data
        print("angle : {:.2f}".format(self.angle))
        # print("angle : {:.2f} x : {:.2f} y : {:.2f}".format(angle, x, y))

    def dynamic_set(self, msg):
        self.dynamic_vel = msg.data
        # print(self.dynamic_vel)


if __name__ == "__main__":
    try:
        kcity_pathtracking = wecar_planner()
    except rospy.ROSInterruptException:
        pass
