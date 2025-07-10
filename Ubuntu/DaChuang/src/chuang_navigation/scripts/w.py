#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
flag2 = 0
flag1 = 0
def NavResultCallback(msg):
    """
    导航结果回调函数
    """
    global flag2
    rospy.logwarn("result %s", msg.data)
    flag2 = 1

if __name__ == "__main__":
    rospy.init_node("wp_node") 

    navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10) # 目标航点名称话题
    res_sub = rospy.Subscriber("/waterplus/navi_result", String, NavResultCallback, queue_size=10) # 接收导航
    rospy.sleep(1)

    navi_msg = String()
    navi_msg.data = "2"
    navi_pub.publish(navi_msg)

    while flag1 == 0:
        
        if flag2 == 1:
            navi_msg = String()
            navi_msg.data = "1"
            navi_pub.publish(navi_msg)
            flag1 = 1
        rospy.sleep(0.1)

    rospy.spin()
