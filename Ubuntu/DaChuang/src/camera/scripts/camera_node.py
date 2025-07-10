#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

class ColorDetector:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('camera_node', anonymous=False)
        self.bridge = CvBridge()

        # 图像发布者
        self.image_pub = rospy.Publisher('detection_result', Image, queue_size=1)
        
        # 控制话题发布者（关键修正：添加self.）
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # 激光雷达订阅
        self.lader_sub = rospy.Subscriber('/scan_filtered', LaserScan, self.scan_callback, queue_size=10)
        
        # 摄像头订阅
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        # 初始化HSV参数
        self.hsv_params = {
            'h_min': 0,
            'h_max': 13,
            's_min': 146,
            's_max': 213,
            'v_min': 138,
            'v_max': 241
        }

        # 图像参数
        self.frameWidth = 640
        self.frameHeight = 480
        self.CENTER_X = self.frameWidth // 2
        self.CENTER_Y = self.frameHeight // 2
        self.stop = False
        self.safe_distance = 0.13
        self.target_distance = 0.1
        self.vel_cmd = Twist()  # 速度指令对象

    def scan_callback(self, msg):
        """激光雷达数据回调"""
        dist = msg.ranges[180]
        rospy.loginfo("front ranges = %f meters", dist)
        
        if dist < self.safe_distance:
            self.stop = True
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = 0.0  # 修正拼写错误
            self.vel_pub.publish(self.vel_cmd)  # 使用self.
            rospy.logwarn_throttle(1, "safety dist %0.2fm", dist)
        else:
            self.stop = False

    def image_callback(self, msg):
        """图像处理回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return
        
        # 初始化速度指令（关键修正：统一使用self.）
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        
        if self.stop:
            self.vel_pub.publish(self.vel_cmd)
            return

        # 图像处理流程
        imgHsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower = np.array([self.hsv_params['h_min'], 
                        self.hsv_params['s_min'],
                        self.hsv_params['v_min']])
        upper = np.array([self.hsv_params['h_max'],
                        self.hsv_params['s_max'],
                        self.hsv_params['v_max']])
        mask = cv2.inRange(imgHsv, lower, upper)
        
        max_contour = None
        max_area = 0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 25 and area > max_area:
                max_area = area
                max_contour = cnt
                
        if max_contour is not None:       
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            center_x = x + w//2
            center_y = y + h//2
            distance = self.CENTER_X - center_x
            rospy.loginfo("Current offset: %d pixels", distance)
            
            if not self.stop:
                if abs(distance) < 5:
                    self.vel_cmd.linear.x = self.target_distance
                else:
                    self.vel_cmd.linear.x = 0.0
                    if distance < 0:
                        self.vel_cmd.angular.z = -0.1
                    else:
                        self.vel_cmd.angular.z = 0.1
                        
                # 统一发布速度指令（优化点）
                self.vel_pub.publish(self.vel_cmd)

if __name__ == '__main__':
    try:
        detector = ColorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass