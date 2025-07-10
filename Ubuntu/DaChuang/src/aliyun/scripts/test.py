#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import paho.mqtt.client as mqtt
import hmac
import hashlib
import base64
import time

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

# 连接回调函数
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to Alibaba Cloud IoT Platform successfully!")
        # 订阅主题
        client.subscribe("/{}/{}/user/pawrx".format(product_key, device_name))
    else:
        print("Failed to connect, return code: {}".format(rc))

# 消息接收回调函数
def on_message(client, userdata, msg):
    print("Received message: {} on topic {}".format(msg.payload, msg.topic))

# 日志回调函数
def on_log(client, userdata, level, buf):
    print("MQTT Log: {}".format(buf))

# 创建 MQTT 客户端，显式设置 client_id
client = mqtt.Client(client_id=client_id)

# 设置回调函数
client.on_connect = on_connect
client.on_message = on_message
client.on_log = on_log  # 启用日志

# 设置用户名和密码
client.username_pw_set(username, password)

# 连接到阿里云 MQTT Broker
print("Connecting to MQTT Broker: {}:{}".format(host, port))
client.connect(host, port, 60)

# 保持连接并处理消息
print("Starting the MQTT client loop...")
client.loop_forever()