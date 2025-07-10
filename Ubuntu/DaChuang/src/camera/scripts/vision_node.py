#!/usr/bin/env python
# coding: utf-8
import rospy
import socket
import json
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

HOST = '127.0.0.1'
PORT = 65432

class VisionTask:
    def __init__(self):
        rospy.init_node('vision_node', anonymous=False)
        self.scan_topic = rospy.get_param('~scan_topic', '/scan_filtered')
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=10)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.safe_distance = 0.13
        self.turn_left_speed = 0.05
        self.turn_right_speed = -0.05
        self.vel_cmd = Twist()
        # 初始让其处于静止状态
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub.publish(self.vel_cmd)
        #小车停止标志位
        self.stop = False

    def scan_callback(self, msg):
        """
        处理雷达数据,当小于安全距离的时候自动停止
        """
        dist = msg.ranges[180]
        if dist < self.safe_distance:
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = 0.0
            self.stop = True
            self.vel_pub.publish(self.vel_cmd)

    def process_latest_json(self, raw_data):
        """从最新数据中提取并处理JSON"""
        try:
            # 从末尾开始查找完整的JSON对象
            end_idx = raw_data.rfind('}')
            if end_idx == -1:
                return  # 没有闭合符，直接丢弃
            
            start_idx = raw_data.rfind('{', 0, end_idx)
            if start_idx == -1:
                return  # 没有起始符，直接丢弃
            
            json_str = raw_data[start_idx:end_idx+1]
            detection_data = json.loads(json_str)
            
            if detection_data.get("detection") == 1:
                # 提取目标信息
                obj = detection_data.get("object", "未知")
                if obj == "QR code":
                    dist = detection_data.get("dist", 0)
                    con = detection_data.get("confidence", 0)
                    if abs(dist) > 25 and self.stop == False:
                        if dist < 0:
                            self.vel_cmd.angular.z = self.turn_right_speed
                            self.vel_cmd.linear.x = 0.01
                        if dist > 0:
                            self.vel_cmd.angular.z = self.turn_left_speed
                            self.vel_cmd.linear.x = 0.01
                    else:
                        self.vel_cmd.angular.z = 0.0
                        self.vel_cmd.linear.x = 0.05
                    if self.stop == False:
                        self.vel_pub.publish(self.vel_cmd)

                    #rospy.loginfo("检测到目标: %s, 横向偏移: %.2f像素, 置信度: %.2f", obj, dist, conf)
            else:
                rospy.loginfo("未检测到目标")
                self.vel_cmd.angular.z = 0.0  # 停止转向
                self.vel_cmd.linear.x = 0.0 # 停止前进
                self.vel_pub.publish(self.vel_cmd)

        except json.JSONDecodeError:
            rospy.logwarn("JSON解析失败，数据可能不完整，已丢弃")
        except Exception as e:
            rospy.logerr("处理数据时出错: %s", str(e))
        

if __name__ == '__main__':
    detector = VisionTask()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((HOST, PORT))
        s.settimeout(0.1)  # 设置非阻塞接收，避免卡在recv
        while not rospy.is_shutdown():
            try:
                data = s.recv(1024)
                if data:
                    raw_str = data.decode('utf-8', errors='ignore')  # 忽略解码错误
                    # 解析JSON数据
                    detection_data = json.loads(raw_str)
                    # 打印JSON数据
                    rospy.loginfo("Received JSON data: %s", detection_data)
                    # 发送确认信息（确保是 unicode 字符串）
                    detector.process_latest_json(raw_str)
                    confirm_msg = u"数据已接收"  # 使用 unicode 字符串
                    s.sendall(confirm_msg.encode('utf-8'))  # 编码为字节流
            except socket.timeout:
                continue  # 非阻塞模式下超时是正常现象
            except Exception as e:
                rospy.logerr("接收数据异常: %s", str(e))
                break
    finally:
        s.close()
    rospy.spin()