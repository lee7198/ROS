#!/usr/bin/env python3
# -*-codingutf-8 -*-

# sub 기반 pub

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan
from morai_msgs.msg import GetTrafficLightStatus
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
from time import *
from time import sleep


class Traffic_control:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        rospy.Subscriber(
            "/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB
        )
        rospy.Subscriber("/ladar2D", LaserScan, self.lidar_CB)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher(
            "/commands/servo/position", Float64, queue_size=1
        )
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()
        self.speed_msg = Float64()
        self.steer_msg = Float64()
        self.traffic_msg = GetTrafficLightStatus()
        self.traffic_flag = 0
        self.perv_signal = 0
        self.signal = 0
        self.cross_flag = 0
        self.img = []
        self.img_flag = False
        self.center_index = 0
        self.standard_line = 0
        self.degree_per_pixel = 0
        self.steer = 0.5
        self.obastacle_flag = False

    def traffic_CB(self, msg: GetTrafficLightStatus):
        self.traffic_msg = msg
        if self.traffic_msg.trafficLightIndex == "SN00002":
            self.signal = self.traffic_msg.trafficLightStatus
            if self.perv_signal != self.signal:
                self.perv_signal = self.signal
            self.traffic_think()

    def traffic_think(self):
        if self.signal == 1:
            # print("red")
            pass
        elif self.signal == 4:
            # print("yellow")
            pass
        elif self.signal == 16:
            # print("green")
            pass
        elif self.signal == 33:
            # print("left")
            pass
        else:
            pass

    def cam_CB(self, msg: CompressedImage):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        (
            self.center_index,
            self.standard_line,
            self.degree_per_pixel,
        ) = self.cam_lane_detection()

        if len(self.img) != 0:
            self.img_flag = True

    def cam_lane_detection(self):
        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.y, self.x = self.img.shape[0:2]

        # yellow
        yellow_lower = [15, 128, 0]
        yellow_upper = [40, 255, 255]
        yellow_img_range = cv2.inRange(
            img_hsv, np.array(yellow_lower), np.array(yellow_upper)
        )

        # white
        white_lower = [0, 0, 192]
        white_upper = [179, 64, 255]
        white_img_range = cv2.inRange(
            img_hsv, np.array(white_lower), np.array(white_upper)
        )
        # merge
        combined_range = cv2.bitwise_or(yellow_img_range, white_img_range)
        filtered_img = cv2.bitwise_and(self.img, self.img, mask=combined_range)

        # warped image
        src_point1 = [0, 420]
        src_point2 = [275, 260]
        src_point3 = [self.x - 275, 260]
        src_point4 = [self.x, 420]
        src_points = np.float32([src_point1, src_point2, src_point3, src_point4])

        dst_point1 = [self.x // 8, 480]
        dst_point2 = [self.x // 8, 0]
        dst_point3 = [self.x // 8 * 7, 0]
        dst_point4 = [self.x // 8 * 7, 480]
        dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        self.warped_img = cv2.warpPerspective(filtered_img, matrix, [self.x, self.y])
        # warped_img = cv2.warpPerspective(
        #     filtered_img, matrix, (x, y), flags=cv2.INTER_LINEAR
        # )
        # binary image
        grayed_img = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        # image thresholding
        bin_img[(grayed_img > 50)] = 255

        hist_x = np.sum(bin_img, axis=0)
        hist_y = np.sum(bin_img, axis=1)
        up_list = hist_y[: self.x // 4 * 3]
        down_list = hist_y[self.x // 2 * 3 :]
        l_hist = np.sum(hist_x[: self.x // 2])
        r_hist = np.sum(hist_x[self.x // 2 :])

        # calc center index
        l_indices = np.where(l_hist > 20)[0]
        r_indices = np.where(r_hist > 20)[0] + 320

        # cross walk
        # ???? why difference the value
        cross_indices = np.where(down_list > 100000)[0] + self.y // 4 * 3
        try:
            cross_threshold = 25
            cross_diff = cross_indices[-1] - cross_indices[0]
            if cross_threshold < cross_diff:
                self.cross_flag = True
                cv2.rectangle(
                    self.warped_img,
                    [0, cross_indices[0]],
                    [self.x, cross_indices[-1]],
                    [0, 255, 0],
                    3,
                )
            else:
                self.cross_flag = False
        except:
            self.cross_flag = False

        indices = np.where(hist_x > 20)[0]

        try:
            if len(l_indices) != 0 and len(r_indices) != 0:
                center_index = (indices[0] + indices[-1]) // 2
                # print("both line")
            elif len(l_indices) != 0 and len(r_indices) == 0:
                center_index = (l_indices[0] + l_indices[-1]) // 2
                # print("left line")
            elif len(l_indices) == 0 and len(r_indices) != 0:
                center_index = (r_indices[0] + r_indices[-1]) // 2
                # print("right line")
        except:
            center_index = self.x // 2
        standard_line = self.x // 2
        degree_per_pixel = 1 / self.x

        return center_index, standard_line, degree_per_pixel

    def lidar_CB(self, msg: LaserScan):
        # os.system("clear")
        self.scan_msg = msg
        self.steer = self.obastacle()

    def obastacle(self):
        degree_min = self.scan_msg.angle_min * 180 / np.pi
        degree_max = self.scan_msg.angle_max * 180 / np.pi
        degree_increment = self.scan_msg.angle_increment * 180 / np.pi

        degrees = [
            degree_min + degree_increment * index
            for index, value in enumerate(self.scan_msg.ranges)  # degree values
        ]
        degree_array = np.array(degrees)
        obstacle_degrees = []
        obstacle_index = []
        for index, value in enumerate(self.scan_msg.ranges):
            if abs(degrees[index]) < 90 and 0 < value < 1:
                obstacle_degrees.append(degrees[index])
                obstacle_index.append(index)
            else:
                pass

        print(len(obstacle_degrees))
        print(obstacle_degrees)
        try:
            right_space = obstacle_index[0] - 180
            left_space = 542 - obstacle_index[-1]

            if left_space < right_space:
                right_degree_avg = (degrees[obstacle_index[0]] + 90) / 2
                degree_avg = right_degree_avg
            else:
                left_degree_avg = (degrees[obstacle_index[-1]] + 90) / 2
                degree_avg = left_degree_avg
            self.obastacle_flag = True
            steer = ((-degree_avg / 90) + 0.5) / 2
        except:
            self.obastacle_flag = False
            degree_avg = 0
            steer = 0.5

        return steer

    def action(self):
        if len(self.img) != 0:
            if self.cross_flag == True and self.signal == 1:
                steer = 0.5
                speed = 0
            else:
                if self.obastacle_flag == True:
                    steer = self.steer
                    speed = 500
                else:
                    # steer = 0.5 + (((center_index - standard_line) * 0.02 / 3.2) / 2)
                    steer = (
                        self.center_index + self.standard_line
                    ) * self.degree_per_pixel
                    steer = 0.5 + steer * 2
                    speed = 1000

            # print("steer: ", Float64(steer))
            self.steer_msg.data = steer
            self.speed_msg.data = speed
            self.speed_pub.publish(self.speed_msg)
            self.steer_pub.publish(self.steer_msg)

            # cv2.imshow("yellow_img_range", yellow_img_range)
            # cv2.imshow("white_img_range", white_img_range)
            # cv2.imshow("combined_range", combined_range)
            cv2.imshow("warped_img", self.warped_img)
            # cv2.imshow("bin_img", bin_img)
            # cv2.imshow("canny_img", canny_img)

            cv2.waitKey(1)


def main():
    try:
        trafic_control = Traffic_control()
        while not rospy.is_shutdown():
            trafic_control.action()
            sleep(0.1)
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
