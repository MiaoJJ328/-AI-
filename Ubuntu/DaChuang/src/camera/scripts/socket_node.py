#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import json
import rospy
from std_msgs.msg import String

# 初始化ROS节点
rospy.init_node('yolo_communicator', anonymous=True)

# 配置客户端
HOST = '127.0.0.1'
PORT = 65432

# 创建 socket 连接
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    # 连接到服务器
    s.connect((HOST, PORT))
    while not rospy.is_shutdown():
        # 接收数据
        data = s.recv(1024)
        if data:
            try: 
                # 解码数据为 unicode 字符串
                data_str = data.decode('utf-8') # 将str(python3当中的bytes类型数据解码为Unicode)
                # 解析JSON数据
                detection_data = json.loads(data_str)
                # 打印JSON数据
                rospy.loginfo("Received JSON data: %s", detection_data)
                # 发送确认信息（确保是 unicode 字符串）
                confirm_msg = u"数据已接收"  # 使用 unicode 字符串
                s.sendall(confirm_msg.encode('utf-8'))  # 编码为字节流
            except UnicodeDecodeError as e:
                rospy.logerr("解码错误: %s", str(e))
            except ValueError as e:  # json.loads 可能抛出 ValueError
                rospy.logerr("JSON解析错误: %s", str(e))
finally:
    # 关闭 socket 连接
    s.close()