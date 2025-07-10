#! /home/hrg/archiconda3/envs/yolov5/bin/python
# -*- coding: utf-8 -*-
import socket
import json

# 配置服务器
HOST = '127.0.0.1'  # 本地地址
PORT = 65432        # 端口号

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    print("服务器已启动，等待连接...")
    conn, addr = s.accept()
    with conn:
        print('连接地址：', addr)
        while True:
            # 假设YOLO检测到的数据为一个字典
            detection_data = {
                "object": "QR code",
                "dist":152,
                "confidence": 0.95
            }
            # 将数据转换为JSON格式
            data = json.dumps(detection_data).encode('utf-8')
            conn.sendall(data)
            # 接收客户端的确认信息
            data = conn.recv(1024)
            if not data:
                break
            print('收到客户端消息：', data.decode('utf-8'))