#! /home/hrg/archiconda3/envs/yolov5/bin/python
# -*- coding: utf-8 -*-
import cv2

# ��ʼ������ͷ
cap = cv2.VideoCapture(0)  # 0 ��ʾĬ������ͷ������ж������ͷ�����Գ��� 1, 2 ��

# �������ͷ�Ƿ�ɹ���
if not cap.isOpened():
    print("can not open video")
    exit()

while True:
    # ��ȡһ֡ͼ��
    ret, frame = cap.read()
    
    # ����Ƿ�ɹ���ȡ֡
    if not ret:
        print("can not read fream")
        break
    
    # ��ʾͼ��
    cv2.imshow('USB Camera', frame)
    
    # ���� 'q' ���˳�ѭ��
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# �ͷ�����ͷ��Դ
cap.release()

# �ر����� OpenCV ����
cv2.destroyAllWindows()