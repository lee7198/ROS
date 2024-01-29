#!/usr/bin/env python3
# -*-codingutf-8 -*-

# sub 기반 pub

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.image_msg = CompressedImage()
        self.bridge = CvBridge()

    def cam_CB(self, msg: CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        y, x = img.shape[0:2]

        # h, s, v = cv2.split(img_hsv)
        # cv2.imshow("h", h)
        # cv2.imshow("s", s)
        # cv2.imshow("v", v)

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
        filtered_img = cv2.bitwise_and(img, img, mask=combined_range)

        # warped image
        src_point1 = [0, 420]
        src_point2 = [260, 260]
        src_point3 = [x - 260, 260]
        src_point4 = [x, 420]
        src_point5 = np.float32([src_point1, src_point2, src_point3, src_point4])

        dst_point1 = [x // 8, 480]
        dst_point2 = [x // 8, 0]
        dst_point3 = [x // 8 * 5, 0]
        dst_point4 = [x // 8 * 5, 480]
        dst_point5 = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])

        matrix = cv2.getPerspectiveTransform(src_point5, dst_point5)
        # warped_img = cv2.warpPerspective(filtered_img, matrix, [x, y])
        warped_img = cv2.warpPerspective(
            filtered_img, matrix, (x, y), flags=cv2.INTER_LINEAR
        )
        # binary image
        grayed_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(grayed_img)
        # image thresholding
        bin_img[(grayed_img > 50)] = 255

        # cv2.imshow("yellow_img_range", yellow_img_range)
        # cv2.imshow("white_img_range", white_img_range)
        # cv2.imshow("combined_range", combined_range)
        # cv2.imshow("warped_img", warped_img)
        cv2.imshow("bin_img", bin_img)

        cv2.waitKey(1)


def main():
    try:
        turtle_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
