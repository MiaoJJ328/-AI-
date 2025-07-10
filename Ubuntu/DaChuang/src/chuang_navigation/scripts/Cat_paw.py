#!/usr/bin/env python
# coding: utf-8
import rospy
import socket
import json
import threading
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

HOST = '127.0.0.1'
PORT = 65432


class CatPaw:
    def __init__(self):
        rospy.init_node('clean_task_node', anonymous=False)

        self.navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10) # 目标航点名称话题
        self.res_sub = rospy.Subscriber("/waterplus/navi_result", String, self.NavResultCallback, queue_size=10) # 接收导航结果的话题

        # 订阅激光雷达数据
        self.scan_topic = rospy.get_param('~scan_topic', '/scan_filtered')
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=10)    
        # 订阅阿里云的消息
        self.ali_sub = rospy.Subscriber("/paw_control", String, self.aliyunCallback, queue_size=10)
        # 用于向阿里云发布消息
        self.ali_pub = rospy.Publisher("/clean_task", String, queue_size=10)
        # 用于发布速度控制指令
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # 安全参数
        self.safe_distance = 0.227
        self.turn_left_speed = 0.05
        self.turn_right_speed = -0.05
        self.vel_cmd = Twist()
        # 初始让其处于静止状态
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub.publish(self.vel_cmd)
        self.stop = False 
        # 事件集合
        self.navigation_even = threading.Event()
        self.look_for_qr_code_even = threading.Event()
        self.wait_for_box_close_even = threading.Event()
        # 检测二维码的状态
        self.QR_state = 0    # 初始状态,未扫描到二维码的状态
        # 线程集合
        self.clean_thread = threading.Thread(target=self.clean_task_state_machine)
        # 线程运行标志位
        self.clean_task_running = False
        self.clean_flag = False
        self._shutdown_flag = False
        self.clean_thread = None
        self.s = None
        self.showdistance = False

    def Init_state(self):
        """
        初始化所有变量
        """
        self.clean_task_running = False
        self.stop = False
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub.publish(self.vel_cmd)
        self.QR_state = 0

    def navigation(self, location):
        """
        发布导航地点函数
        """
        navi_msg = String()
        navi_msg.data = location
        self.navi_pub.publish(navi_msg)
        rospy.logwarn("向目标点%s导航中...", location)


    def NavResultCallback(self, msg):
        """
        接收导航结果话题，发布目标航点名称话题
        """
        if msg.data != "done":
            rospy.logerr("导航失败！！！！")
        else:
            rospy.logwarn("导航成功！")
            self.navigation_even.set() # 导航完成事件置位
            self.showdistance = True # 显示距离标志位置位

        if self.clean_flag == True:
            self.stop_clean_task()
            self.clean_flag = False


    def scan_callback(self, msg):
        """
        处理雷达数据,当小于安全距离的时候自动停止
        """
        dist = msg.ranges[180]
        if self.showdistance:
            rospy.loginfo("正前方距离: %.2f 米", dist)
        if dist < self.safe_distance:
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = 0.0
            self.stop = True
            self.vel_pub.publish(self.vel_cmd)


    def start_clean_task(self):
            """启动清理任务线程"""
            # 如果线程未运行或已终止，则创建新线程
            if self.clean_thread is None or not self.clean_thread.is_alive():
                self.clean_task_running = True  # 设置运行标志
                self.clean_thread = threading.Thread(target=self.clean_task_state_machine)
                self.clean_thread.start()
                rospy.loginfo("清理线程已启动")
            else:
                rospy.logwarn("清理线程已在运行")

    def stop_clean_task(self):
        """停止清理任务"""
        if self.clean_thread is not None and self.clean_thread.is_alive():
            self.clean_task_running = False
            self.navigation_even.set()              # 唤醒导航等待
            self.wait_for_box_close_even.set()      # 唤醒箱体关闭等待
            self.clean_thread.join(timeout=2.0)  # 等待线程终止，最长等待2秒
            if self.clean_thread.is_alive():
                rospy.logwarn("清理线程未正常退出，强制终止")
            self.clean_thread = None  # 重置线程对象
            rospy.loginfo("清理线程已停止")


    def aliyunCallback(self, msg):
        """
        处理阿里云接收到的数据
        """
        data = json.loads(msg.data)
        rospy.loginfo("Received JSON data: {}".format(data))
        if "paw_clean" in data:
            value = data["paw_clean"]
            if value == 1:
                self.start_clean_task()  # 开启清理线程
        if "box_close" in data:
            value = data["box_close"]
            if value == 1:
                self.wait_for_box_close_even.set() # 箱子关闭就回开始后退

    
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
                # 发布检测到QR_code的消息
                # 停止转向
                if self.QR_state == 0:
                    self.vel_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.vel_cmd)
                    self.QR_state = 1 # 进入到状态2，不会再因为扫描不到到而影响转向
                    
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
                if self.QR_state == 0:
                    self.vel_cmd.angular.z = -0.01  # 转向寻找二维码
                    self.vel_cmd.linear.x = 0.0 # 停止前进
                    self.vel_pub.publish(self.vel_cmd)

        except json.JSONDecodeError:
            rospy.logwarn("JSON解析失败，数据可能不完整，已丢弃")
        except Exception as e:
            rospy.logerr("处理数据时出错: %s", str(e))


    def socket_handle(self):
        """
        socket通信处理函数
        """
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 每次创建新 Socket
            self.s.connect((HOST, PORT))
            self.s.settimeout(0.1)
            while not rospy.is_shutdown() and self.clean_task_running and not self.stop:
                try:
                    data = self.s.recv(1024)
                    if data:
                        raw_str = data.decode('utf-8', errors='ignore')  # 忽略解码错误
                        # 解析JSON数据
                        detection_data = json.loads(raw_str)
                        # 打印JSON数据
                        rospy.loginfo("Received JSON data: %s", detection_data)
                        # 发送确认信息（确保是 unicode 字符串）
                        self.process_latest_json(raw_str)
                        confirm_msg = u"数据已接收"  # 使用 unicode 字符串
                        self.s.sendall(confirm_msg.encode('utf-8'))  # 编码为字节流
                        # 当小车停止的时候关闭消息接收
                        if self.stop == True:
                            self.s.close()
                            break
                except socket.timeout:
                    continue  # 非阻塞模式下超时是正常现象
                except Exception as e:
                    rospy.logerr("接收数据异常: %s", str(e))
                    break
        finally:
            self.s.close()


    def backward(self):
        """
        倒退程序
        """
        start_time = rospy.Time.now()  # 记录起始时间
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 8:
            self.vel_cmd.linear.x = -0.1  # 设置线速度为负值，表示倒退
            self.vel_cmd.angular.z = 0.0  # 角速度为0，表示不旋转
            self.vel_pub.publish(self.vel_cmd)  # 发布速度命令
            rospy.loginfo("倒退中...")
            rospy.sleep(0.1)  # 每隔0.1秒检查一次时间

    def clean_task_state_machine(self):
        """
        清理线程
        """
        if not self.clean_task_running:
            return

        # 进入搭配导航的状态
        self.navigation("2")
        self.navigation_even.wait() # 等待导航完成
        self.navigation_even.clear() # 清除导航完成事件
        # Socket通信处理
        self.socket_handle()
        if self.stop == True:
            self.ali_pub.publish(String("box_open"))
        # 等待箱子关闭事件结束
        self.wait_for_box_close_even.wait()
        self.wait_for_box_close_even.clear()
        self.showdistance = False # 显示距离标志位置位
        # 开始执行倒退程序
        self.backward()
        # 导航到起点
        self.navigation("1")
        self.clean_flag = True # 表示清理完成
        self.navigation_even.wait() # 等待导航完成
        self.navigation_even.clear()

        # 所有状态都回复到起始的样子
        self.Init_state()

    def shutdown(self):
        """
        节点退出时安全终止所有线程和资源
        """
        rospy.loginfo("节点关闭中，清理资源...")
        self._shutdown_flag = True
        self.stop_clean_task()
        # 强制停止机器人运动
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub.publish(self.vel_cmd)


if __name__ == '__main__':
    clean_task = CatPaw()
    rospy.on_shutdown(clean_task.shutdown)
    rospy.spin()