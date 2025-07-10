import cv2
url = "http://192.168.92.146/mjpeg/1"
cap = cv2.VideoCapture(url)
if not cap.isOpened():
    print("fail")
else:
    print("sucess")
cap.release()