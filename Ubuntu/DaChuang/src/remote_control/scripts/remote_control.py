#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import paho.mqtt.client as mqtt
import rospy
import json
from geometry_msgs.msg import Twist

# 阿里云IoT三元组信息
ALIYUN_MQTT = {
    "HOST": "iot-06z00godbmkjb42.mqtt.iothub.aliyuncs.com",  # 阿里云IoT的Endpoint
    "PORT": 1883,
    "CLIENT_ID": "k1r45F1HfgR.Cat_paw|securemode=2,signmethod=hmacsha256,timestamp=1740191888477|",
    "USERNAME": "Cat_paw&k1r45F1HfgR",
    "PASSWORD": "17db7c4ee0826bf97e7d87320963a170acc8a2a9e5e59019cbb1c26f2304db5e",
    "TOPIC_SUB": "/k1r45F1HfgR/Cat_paw/user/pawrx"
}
# 全局控制变量
remote_control_enabled = False
current_speed = Twist()
cmd_vel_pub = None

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Connected to Alibaba Cloud IoT")
        client.subscribe(ALIYUN_MQTT["TOPIC_SUB"])
    else:
        rospy.logerr("Connection failed code: %d", rc)

def on_message(client, userdata, msg):
    global remote_control_enabled, current_speed, car_stop_enable
    try:
        payload = json.loads(msg.payload.decode())
        rospy.loginfo("Received: %s", payload)

        # 处理远程控制开关
        if "remotecontrol" in payload:
            remote_control_enabled = (payload["remotecontrol"] == 1)
            rospy.logwarn("Remote control %s", "ENABLED" if remote_control_enabled else "DISABLED")
            if not remote_control_enabled:
                current_speed = Twist()  # 停止小车
                
        # 紧急停止
        if "car_stop" in payload:
            car_stop_enabled = (payload["car_stop"] == 1)
            rospy.logwarn("car_stop_enabled %s", "ENABLED" if car_stop_enabled else "DISABLED")
            current_speed.linear.x = 0
            current_speed.angular.z = 0
            cmd_vel_pub.publish(current_speed)

        # 处理速度指令（仅在遥控模式生效）
        if remote_control_enabled:
            if "car_linespeed" in payload:
                current_speed.linear.x = float(payload["car_linespeed"])
            if "car_anglespeed" in payload:
                current_speed.angular.z = float(payload["car_anglespeed"])
            
            # 实时发布速度
            cmd_vel_pub.publish(current_speed)

    except json.JSONDecodeError:
        rospy.logerr("Invalid JSON format")
    except KeyError as e:
        rospy.logwarn("Unexpected message key: %s", e)
    except ValueError:
        rospy.logerr("Invalid speed value type")

def main():
    global cmd_vel_pub
    rospy.init_node('aliyun_remote_control')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    client = mqtt.Client(client_id=ALIYUN_MQTT["CLIENT_ID"])
    client.username_pw_set(ALIYUN_MQTT["USERNAME"], ALIYUN_MQTT["PASSWORD"])
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(ALIYUN_MQTT["HOST"], ALIYUN_MQTT["PORT"], 60)
    rospy.loginfo("Starting MQTT loop...")
    client.loop_start()

    # 保持ROS节点运行并持续发送速度（防止断连）
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if remote_control_enabled:
            cmd_vel_pub.publish(current_speed)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass