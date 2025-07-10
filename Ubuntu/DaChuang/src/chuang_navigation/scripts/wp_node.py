#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String
import json
import threading

paw_destination = "None"
paw_location = "1"  # 默认的位置在起始处
navigation_state = 0 # 0表示静默状态 1表示导航中 2表示导航成功 3表示导航失败
paw_clean = 0 # 开启清洁
event_learm_done = threading.Event() # 用于通知事件完成 
event_box_done = threading.Event() # 用于等待完成开关收纳盒
event_navigation_done = threading.Event() # 用于表示完成了导航
event_clean_done = threading.Event() # 清洁线程开启
thread_running = True
def navigation(location):
    """
    发布导航地点函数
    """
    navi_msg = String()
    navi_msg.data = location
    navi_pub.publish(navi_msg)

def learm(data):
    """
    控制机械臂函数
    """
    learm_msg = String()
    learm_msg.data = data
    serial_pub.publish(learm_msg)

def learm_callback(msg):
    """
    机械臂执行结果回调函数
    """
    if msg.data == "done":
        event_learm_done.set() # 机械臂动作执行完成


def NavResultCallback(msg):
    """
    导航结果回调函数
    """
    global navigation_state
    global paw_location
    global paw_destination
    if msg.data != "done":
        rospy.logerr("nav_results = failed")
        navigation_state = 3
    else:
        rospy.logwarn("nav_results = %s", msg.data)
        paw_location = paw_destination
        navigation_state = 2
        event_navigation_done.set() # 设置导航事件表示成功

def aliyunCallback(msg):
    """
    阿里云回调函数
    """
    global paw_location
    global paw_destination
    global paw_clean
    data = json.loads(msg.data)
    rospy.loginfo("Received JSON data: {}".format(data))
    if "paw_location" in data:
        value = data["paw_location"]
        if value != paw_location:
            paw_destination = value # 更新导航目标地点
            navigation(paw_destination) # 开始向目标点导航
        else:
            rospy.logwarn("stay still!!!")
    elif "paw_clean" in data:
        paw_clean = data["paw_clean"]
        if paw_clean == 1:
            event_clean_done.set()
    elif "box_finish" in data:
        value = data["box_finish"]
        if value == 1:
            event_box_done.set()


def clean_threadfunc():
    """
    清洁函数
    """
    global paw_destination
    while thread_running:
        # 等待接收到clean命令
        event_clean_done.wait()
        event_clean_done.clear()
        if not thread_running:
            rospy.logwarn("thread shutdown")
            break
        rospy.loginfo("begin cleanning ...")
        
        # 开始向目标点导航
        navigation("3") 
        event_navigation_done.wait()
        event_navigation_done.clear()
        rospy.loginfo("arrived position 3!")
        # 阻塞直到事件被重置（在这里等待清洁完成）
        learm("pickup")
        event_learm_done.wait() 
        event_learm_done.clear() # 重置事件
        rospy.loginfo("learm pickup finished!")
        # 导航到马桶那里
        navigation("2") 
        event_navigation_done.wait()
        event_navigation_done.clear()
        rospy.loginfo("arrived position 2!")
        #倾倒动作
        learm("pour")
        event_box_done.wait() # 等待盒子完成
        event_box_done.clear()
        #再次返回将其放回
        navigation("3")
        event_navigation_done.wait()
        event_navigation_done.clear()
        rospy.loginfo("arrived position 3!")
        learm("setdown")
        event_learm_done.wait()
        event_learm_done.clear()
        rospy.loginfo("learm setdown!")
        #机械臂回到初始位置
        learm("init_pos")
        event_learm_done.wait()
        event_learm_done.clear()
        rospy.loginfo("learn return init!")
        # 返回到初始位置
        navigation("1")
        event_navigation_done.wait()
        event_navigation_done.clear()
        rospy.loginfo("got position 1!")

def cleanup():
    global thread_running
    rospy.loginfo("Cleaning up resources...")
    thread_running = False
    clean_thread.join() # 等待线程结束
    rospy.loginfo("Thread terminated.")



if __name__ == "__main__":
    rospy.init_node("wp_node") 

    ali_sub = rospy.Subscriber("/paw_control", String, aliyunCallback, queue_size=10)
    navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10) # 目标航点名称话题
    res_sub = rospy.Subscriber("/waterplus/navi_result", String, NavResultCallback, queue_size=10) # 接收导航结果的话题
    serial_pub = rospy.Publisher("/learm_control", String, queue_size=10) # 创建一个控制机械臂的话题
    serial_sub = rospy.Subscriber("learm_result", String, learm_callback, queue_size=10)
    rospy.sleep(1)

    rospy.on_shutdown(cleanup)  # 注册 cleanup 方法为关闭回调
    # 创建清洁线程
    clean_thread = threading.Thread(target=clean_threadfunc)
    clean_thread.start()

    rate = rospy.Rate(5)  # 设置目标频率
    while not rospy.is_shutdown():
        # 在此执行需要周期性运行的逻辑
        # 例如状态机更新、数据发布等
        try:
            rate.sleep()  # 根据设定频率休眠
        except rospy.ROSInterruptException:
            rospy.loginfo("Main loop interrupted")
