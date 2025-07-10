#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import paho.mqtt.client as mqtt
import rospy
import json
from std_msgs.msg import String

# 阿里云 IoT 平台的三元组信息
product_key = "k1r45F1HfgR"  # 替换为你的 ProductKey
device_name = "Cat_paw"      # 替换为你的 DeviceName
device_secret = "abe39a6b29082e1cf66dba521975755d"  # 替换为你的 DeviceSecret

# MQTT 连接的 clientId、用户名和密码
client_id = "k1r45F1HfgR.Cat_paw|securemode=2,signmethod=hmacsha256,timestamp=1740191888477|"
username = "Cat_paw&k1r45F1HfgR"
password = "17db7c4ee0826bf97e7d87320963a170acc8a2a9e5e59019cbb1c26f2304db5e"

# MQTT Broker 地址和端口
host = "iot-06z00godbmkjb42.mqtt.iothub.aliyuncs.com"  # 阿里云 MQTT Broker 地址
port = 1883  # 非加密端口，如果使用加密连接，请改为 8883

client = None 

# 连接回调函数
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Connected to Alibaba Cloud IoT Platform successfully!")
        # 订阅主题
        client.subscribe("/{}/{}/user/pawrx".format(product_key, device_name))
    else:
        rospy.logerr("Failed to connect, return code: {}".format(rc))

# 消息接收回调函数
def on_message(client, userdata, msg):
    try:
        # 获取消息内容（JSON 字符串）
        payload = msg.payload.decode('utf-8')
        rospy.loginfo("Received JSON message: {}".format(payload))

        # 将 JSON 字符串直接发布到 ROS 话题
        pub.publish(payload)
        rospy.loginfo("Published to /paw_control: {}".format(payload))
    except Exception as e:
        rospy.logerr("Error processing message: {}".format(e))

# 日志回调函数
def on_log(client, userdata, level, buf):
    rospy.loginfo("MQTT Log: {}".format(buf))


def clean_task_callback(msg):
    global client
    # 接收到清洁任务指令，发布到阿里云 IoT 平台
    if msg.data == "box_open":
        payload = json.dumps({"box_open": 1})
        client.publish("/{}/{}/user/pawtx".format(product_key, device_name), payload)


def main():
    # 初始化 ROS 节点和发布器
    rospy.init_node('aliyun_node')
    global pub, client
    pub = rospy.Publisher('/paw_control', String, queue_size=10)
    clean_task_sub = rospy.Subscriber('/clean_task', String, clean_task_callback, queue_size=10)

    # 创建 MQTT 客户端，显式设置 client_id
    client = mqtt.Client(client_id=client_id)

    # 设置回调函数
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_log = on_log  # 启用日志

    # 设置用户名和密码
    client.username_pw_set(username, password)

    # 连接到阿里云 MQTT Broker
    rospy.loginfo("Connecting to MQTT Broker: {}:{}".format(host, port))
    client.connect(host, port, 60)

    # 保持连接并处理消息
    rospy.loginfo("Starting the MQTT client loop...")
    client.loop_start()

    # 保持 ROS 节点运行
    rospy.spin()

if __name__ == "__main__":
    main()
