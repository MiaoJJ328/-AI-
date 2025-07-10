#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import paho.mqtt.client as mqtt
import hmac
import hashlib
import base64
import time

# ������ IoT ƽ̨����Ԫ����Ϣ
product_key = "k1r45F1HfgR"  # �滻Ϊ��� ProductKey
device_name = "Cat_paw"      # �滻Ϊ��� DeviceName
device_secret = "abe39a6b29082e1cf66dba521975755d"  # �滻Ϊ��� DeviceSecret

# MQTT ���ӵ� clientId���û���������
client_id = "k1r45F1HfgR.Cat_paw|securemode=2,signmethod=hmacsha256,timestamp=1740191888477|"
username = "Cat_paw&k1r45F1HfgR"
password = "17db7c4ee0826bf97e7d87320963a170acc8a2a9e5e59019cbb1c26f2304db5e"

# MQTT Broker ��ַ�Ͷ˿�
host = "iot-06z00godbmkjb42.mqtt.iothub.aliyuncs.com"  # ������ MQTT Broker ��ַ
port = 1883  # �Ǽ��ܶ˿ڣ����ʹ�ü������ӣ����Ϊ 8883

# ���ӻص�����
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to Alibaba Cloud IoT Platform successfully!")
        # ��������
        client.subscribe("/{}/{}/user/pawrx".format(product_key, device_name))
    else:
        print("Failed to connect, return code: {}".format(rc))

# ��Ϣ���ջص�����
def on_message(client, userdata, msg):
    print("Received message: {} on topic {}".format(msg.payload, msg.topic))

# ��־�ص�����
def on_log(client, userdata, level, buf):
    print("MQTT Log: {}".format(buf))

# ���� MQTT �ͻ��ˣ���ʽ���� client_id
client = mqtt.Client(client_id=client_id)

# ���ûص�����
client.on_connect = on_connect
client.on_message = on_message
client.on_log = on_log  # ������־

# �����û���������
client.username_pw_set(username, password)

# ���ӵ������� MQTT Broker
print("Connecting to MQTT Broker: {}:{}".format(host, port))
client.connect(host, port, 60)

# �������Ӳ�������Ϣ
print("Starting the MQTT client loop...")
client.loop_forever()