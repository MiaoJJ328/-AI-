#! /home/hrg/archiconda3/envs/yolov5/bin/python
# -*- coding: utf-8 -*-
import cv2

# 初始化摄像头
cap = cv2.VideoCapture(0)  # 0 表示默认摄像头，如果有多个摄像头，可以尝试 1, 2 等

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("can not open video")
    exit()

while True:
    # 读取一帧图像
    ret, frame = cap.read()
    
    # 检查是否成功读取帧
    if not ret:
        print("can not read fream")
        break
    
    # 显示图像
    cv2.imshow('USB Camera', frame)
    
    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源
cap.release()

# 关闭所有 OpenCV 窗口
cv2.destroyAllWindows()