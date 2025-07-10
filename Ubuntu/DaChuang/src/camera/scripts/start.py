#!/usr/bin/env python
# coding: utf-8

import rospy
import os
import subprocess
import signal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomResetter:
    def __init__(self):
        # 创建静态 TF 广播器
        self.tf_broadcaster = StaticTransformBroadcaster()
        # 初始化并发布一次静态变换
        self.publish_static_tf()

    def publish_static_tf(self):
        """发布一次静态 TF 对齐坐标系"""
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "odom"
        static_transform.child_frame_id = "base_footprint"
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.w = 1.0  # 无旋转
        self.tf_broadcaster.sendTransform(static_transform)
        rospy.loginfo("已发布静态 TF，对齐 odom 和 base_footprint!")

if __name__ == '__main__':
    try:
        # ---------------------- 初始化ROS节点 ----------------------
        rospy.init_node('startup_script', anonymous=True)
          
        # ---------------------- 重启Web服务 ----------------------
        password = "123456"
        command = "sudo -S systemctl restart julab_web.service"
        process1 = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process1.communicate(input=password.encode())

        # ---------------------- 启动其他ROS节点 ----------------------
        #process3 = subprocess.Popen(["rosrun", "camera", "detection.py"])
        process3 = subprocess.Popen(["rosrun", "camera", "socket_test.py"])

        # ---------------------- 等待子进程结束 ----------------------
        process3.wait()

    except KeyboardInterrupt:
        print("检测到 Ctrl + C，正在终止子进程...")
        # 终止所有子进程
        process1.terminate()
        process3.terminate()
        print("子进程已终止")