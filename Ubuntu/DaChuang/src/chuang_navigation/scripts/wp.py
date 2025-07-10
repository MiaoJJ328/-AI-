#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String
import json

# 全局状态变量
paw_destination = "None"
paw_location = "1"
navigation_state = 0
paw_clean = 0
clean_step = 0  # 清洁流程步骤控制
learm_action_done = False
navigation_done = False
box_done = False

# ROS发布器
navi_pub = None
serial_pub = None

def navigation(location):
    global navigation_done
    navigation_done = False
    navi_pub.publish(String(location))

def learm_control(action):
    global learm_action_done
    learm_action_done = False
    serial_pub.publish(String(action))

def aliyunCallback(msg):
    global paw_destination, paw_location, paw_clean, clean_step, box_done
    data = json.loads(msg.data)
    
    if "paw_location" in data:
        value = data["paw_location"]
        if value != paw_location and clean_step == 0:  # 仅在非清洁状态响应
            paw_destination = value
            navigation(value)
    elif "paw_clean" in data:
        if data["paw_clean"] == 1 and clean_step == 0:
            clean_step = 1  # 启动清洁流程
    elif "box_finish" in data:
        if data["box_finish"] == 1:
            box_done = True

def NavResultCallback(msg):
    global navigation_done, paw_location, paw_destination
    if msg.data == "done":
        paw_location = paw_destination
        navigation_done = True
    else:
        navigation_done = False

def learm_callback(msg):
    global learm_action_done
    if msg.data == "done":
        learm_action_done = True

def main_loop():
    global clean_step, box_done, learm_action_done, navigation_done
    rate = rospy.Rate(5)  # 10Hz主循环
    
    while not rospy.is_shutdown():
        # 清洁流程状态机
        if clean_step > 0:
            if clean_step == 1:  # 步骤1：导航到位置3
                navigation("3")
                clean_step = 2
                rospy.loginfo("Navigating to position3...")
                
            elif clean_step == 2 and navigation_done:  # 到达位置3
                learm_control("pickup")
                clean_step = 3
                rospy.loginfo("Picking up...")
                
            elif clean_step == 3 and learm_action_done:  # 拾取完成
                navigation("2")
                clean_step = 4
                rospy.loginfo("Navigating to position2...")
                
            elif clean_step == 4 and navigation_done:  # 到达位置2
                clean_step = 5
                rospy.loginfo("Waiting for box...")
                
            elif clean_step == 5 and box_done:  # 盒子操作完成
                navigation("3")
                box_done = False
                clean_step = 6
                rospy.loginfo("Returning to position3...")
                
            elif clean_step == 6 and navigation_done:  # 返回位置3
                learm_control("setdown")
                clean_step = 7
                rospy.loginfo("Setting down...")
                
            elif clean_step == 7 and learm_action_done:  # 放置完成
                learm_control("init_pos")
                clean_step = 8
                rospy.loginfo("Returning to init...")
                
            elif clean_step == 8 and learm_action_done:  # 机械臂复位
                navigation("1")
                clean_step = 9
                rospy.loginfo("Going home...")
                
            elif clean_step == 9 and navigation_done:  # 返回初始位置
                clean_step = 0
                rospy.loginfo("Clean complete!")
        
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("wp_node")
    
    # 初始化发布器
    navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10)
    serial_pub = rospy.Publisher("/learm_control", String, queue_size=10)
    
    # 订阅器
    rospy.Subscriber("/paw_control", String, aliyunCallback, queue_size=10)
    rospy.Subscriber("/waterplus/navi_result", String, NavResultCallback, queue_size=10)
    rospy.Subscriber("learm_result", String, learm_callback, queue_size=10)
    
    rospy.loginfo("WP Node Started")
    main_loop()