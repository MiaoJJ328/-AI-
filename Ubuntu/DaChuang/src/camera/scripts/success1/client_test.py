#!/usr/bin/env python
# -*- coding: gbk -*-
import rospy
import socket
import json
import threading
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
HOST = '127.0.0.1'
PORT = 65432


class CatPaw:
    def __init__(self):
        rospy.init_node('clean_task_node', anonymous=False)

        self.navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10) # Ŀ�꺽�����ƻ���
        self.res_sub = rospy.Subscriber("/waterplus/navi_result", String, self.NavResultCallback, queue_size=10) # ���յ�������Ļ���

        # ���ļ����״�����
        self.scan_topic = rospy.get_param('~scan_topic', '/scan_filtered')
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=10)    
        # ���İ����Ƶ���Ϣ
        self.ali_sub = rospy.Subscriber("/paw_control", String, self.aliyunCallback, queue_size=10)
        # ���������Ʒ�����Ϣ
        self.ali_pub = rospy.Publisher("/clean_task", String, queue_size=10)
        # ���ڷ����ٶȿ���ָ��
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # ��ȫ����
        self.safe_distance = 0.227
        self.turn_left_speed = 0.05
        self.turn_right_speed = -0.05
        self.vel_cmd = Twist()
        # ��ʼ���䴦�ھ�ֹ״̬
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub.publish(self.vel_cmd)
        self.stop = False 
        # �¼�����
        self.navigation_even = threading.Event()
        self.look_for_qr_code_even = threading.Event()
        self.wait_for_box_close_even = threading.Event()
        # ����ά���״̬
        self.QR_state = 0    # ��ʼ״̬,δɨ�赽��ά���״̬
        # �̼߳���
        self.clean_thread = threading.Thread(target=self.clean_task_state_machine)
        # �߳����б�־λ
        self.clean_task_running = False
        self.clean_flag = False
        self._shutdown_flag = False
        self.clean_thread = None
        self.s = None
        self.showdistance = False
        ################################################
        # �������Ϳ�����ر���
        self.send_flag = True  # True:����cat_detection, False:����qr_code_detection
        self.sending_thread = None
        self.sending_active = False
        self.socket_lock = threading.Lock()
        
        # ���������߳�
        self.start_sending()
        ################################################
    def start_sending(self):
        """���������߳�"""
        if self.sending_thread is None or not self.sending_thread.is_alive():
            self.sending_active = True
            self.sending_thread = threading.Thread(target=self.send_detection_status)
            self.sending_thread.start()
            rospy.loginfo("���״̬�����߳�������")

    def stop_sending(self):
        """ֹͣ�����߳�"""
        self.sending_active = False
        if self.sending_thread and self.sending_thread.is_alive():
            self.sending_thread.join()
            rospy.loginfo("���״̬�����߳���ֹͣ")

    def send_detection_status(self):
        """�������ͼ��״̬"""
        while self.sending_active and not rospy.is_shutdown():
            try:
                with self.socket_lock:
                    # ��ʼ��socket�����δ�������ѹرգ�
                    if not hasattr(self, 's') or (self.s is None) or (self.s.fileno() == -1):  # ʹ��fileno�����Ч��
                        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.s.connect((HOST, PORT))
                        rospy.loginfo("�ɹ����ӵ������: %s:%d" % (HOST, PORT))
                    
                    # ���췢����Ϣ
                    if self.send_flag:
                        msg = {"cat_detection": True}
                    else:
                        msg = {"qr_code_detection": True}
                    
                    # ����JSON����
                    self.s.sendall(json.dumps(msg).encode('utf-8'))
                    rospy.loginfo("�ѷ��ͼ��״̬: %s" % msg)
                
                time.sleep(0.5)  # ���Ʒ���Ƶ��
                
            except Exception as e:
                rospy.logerr("���ʹ���: %s" % str(e))
                time.sleep(1)
                try:
                    if hasattr(self, 's') and self.s is not None:
                        self.s.close()
                        self.s = None  # ����socket����
                except:
                    pass
        
    def Init_state(self):
        """
        ��ʼ�����б���
        """
        self.clean_task_running = False
        self.stop = False
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub.publish(self.vel_cmd)
        self.QR_state = 0

    def navigation(self, location):
        """
        ���������ص㺯��
        """
        navi_msg = String()
        navi_msg.data = location
        self.navi_pub.publish(navi_msg)
        rospy.logwarn("��Ŀ���%s������...", location)


    def NavResultCallback(self, msg):
        """
        ���յ���������⣬����Ŀ�꺽�����ƻ���
        """
        if msg.data != "done":
            rospy.logerr("����ʧ�ܣ�������")
        else:
            rospy.logwarn("�����ɹ���")
            self.navigation_even.set() # ��������¼���λ
            self.showdistance = True # ��ʾ�����־λ��λ

        if self.clean_flag == True:
            self.stop_clean_task()
            self.clean_flag = False


    def scan_callback(self, msg):
        """
        �����״�����,��С�ڰ�ȫ�����ʱ���Զ�ֹͣ
        """
        dist = msg.ranges[180]
        if self.showdistance:
            rospy.loginfo("��ǰ������: %.2f ��", dist)
        if dist < self.safe_distance:
            self.vel_cmd.linear.x = 0.0
            self.vel_cmd.angular.z = 0.0
            self.stop = True
            self.vel_pub.publish(self.vel_cmd)


    def start_clean_task(self):
            """�������������߳�"""
            # ����߳�δ���л�����ֹ���򴴽����߳�
            if self.clean_thread is None or not self.clean_thread.is_alive():
                self.clean_task_running = True  # �������б�־
                self.clean_thread = threading.Thread(target=self.clean_task_state_machine)
                self.clean_thread.start()
                rospy.loginfo("�����߳�������")
            else:
                rospy.logwarn("�����߳���������")

    def stop_clean_task(self):
        """ֹͣ��������"""
        if self.clean_thread is not None and self.clean_thread.is_alive():
            self.clean_task_running = False
            self.navigation_even.set()              # ���ѵ����ȴ�
            self.wait_for_box_close_even.set()      # ��������رյȴ�
            self.clean_thread.join(timeout=2.0)  # �ȴ��߳���ֹ����ȴ�2��
            if self.clean_thread.is_alive():
                rospy.logwarn("�����߳�δ�����˳���ǿ����ֹ")
            self.clean_thread = None  # �����̶߳���
            rospy.loginfo("�����߳���ֹͣ")


    def aliyunCallback(self, msg):
        """
        �������ƽ��յ������ݣ��޸Ĳ��֣�
        """
        data = json.loads(msg.data)
        rospy.loginfo("״̬: %s" % msg)
        rospy.loginfo("Received JSON data: {}".format(data))
        if "paw_clean" in data:
            value = data["paw_clean"]
            if value == 1:
                self.send_flag = False  # �л�Ϊ���Ͷ�ά����
                self.start_clean_task()  # ���������߳�
                rospy.logwarn("[״̬���] �����ά����ģʽ��ֹͣ��������")
        if "box_close" in data:
            value = data["box_close"]
            if value == 1:
                self.wait_for_box_close_even.set() # ���ӹرվͻؿ�ʼ����

    
    def process_latest_json(self, raw_data):
        """��������������ȡ������JSON"""
        try:
            # ��ĩβ��ʼ����������JSON����
            end_idx = raw_data.rfind('}')
            if end_idx == -1:
                return  # û�бպϷ���ֱ�Ӷ���
            
            start_idx = raw_data.rfind('{', 0, end_idx)
            if start_idx == -1:
                return  # û����ʼ����ֱ�Ӷ���
            
            json_str = raw_data[start_idx:end_idx+1]
            detection_data = json.loads(json_str)
            
            if detection_data.get("detection") == 1:
                # ��ȡĿ����Ϣ
                # ������⵽QR_code����Ϣ
                # ֹͣת��
                if self.QR_state == 0:
                    self.vel_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.vel_cmd)
                    self.QR_state = 1 # ���뵽״̬2����������Ϊɨ�費������Ӱ��ת��
                    
                obj = detection_data.get("object", "δ֪")
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

                    #rospy.loginfo("��⵽Ŀ��: %s, ����ƫ��: %.2f����, ���Ŷ�: %.2f", obj, dist, conf)
            else:
                rospy.loginfo("δ��⵽Ŀ��")
                if self.QR_state == 0:
                    self.vel_cmd.angular.z = -0.01  # ת��Ѱ�Ҷ�ά��
                    self.vel_cmd.linear.x = 0.0 # ֹͣǰ��
                    self.vel_pub.publish(self.vel_cmd)

        except json.JSONDecodeError:
            rospy.logwarn("JSON����ʧ�ܣ����ݿ��ܲ��������Ѷ���")
        except Exception as e:
            rospy.logerr("��������ʱ����: %s", str(e))


    def socket_handle(self):
        """
        socketͨ�Ŵ�����
        """
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # ÿ�δ����� Socket
            self.s.connect((HOST, PORT))
            self.s.settimeout(0.1)
            while not rospy.is_shutdown() and self.clean_task_running and not self.stop:
                try:
                    data = self.s.recv(1024)
                    if data:
                        raw_str = data.decode('utf-8', errors='ignore')  # ���Խ������
                        # ����JSON����
                        detection_data = json.loads(raw_str)
                        # ��ӡJSON����
                        rospy.loginfo("Received JSON data: %s", detection_data)
                        # ����ȷ����Ϣ��ȷ���� unicode �ַ�����
                        self.process_latest_json(raw_str)
                        confirm_msg = u"�����ѽ���"  # ʹ�� unicode �ַ���
                        self.s.sendall(confirm_msg.encode('utf-8'))  # ����Ϊ�ֽ���
                        # ��С��ֹͣ��ʱ��ر���Ϣ����
                        if self.stop == True:
                            self.s.close()
                            break
                except socket.timeout:
                    continue  # ������ģʽ�³�ʱ����������
                except Exception as e:
                    rospy.logerr("���������쳣: %s", str(e))
                    break
        finally:
            self.s.close()


    def backward(self):
        """
        ���˳���
        """
        start_time = rospy.Time.now()  # ��¼��ʼʱ��
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 8:
            self.vel_cmd.linear.x = -0.1  # �������ٶ�Ϊ��ֵ����ʾ����
            self.vel_cmd.angular.z = 0.0  # ���ٶ�Ϊ0����ʾ����ת
            self.vel_pub.publish(self.vel_cmd)  # �����ٶ�����
            rospy.loginfo("������...")
            rospy.sleep(0.1)  # ÿ��0.1����һ��ʱ��

    def clean_task_state_machine(self):
        """
        �����߳�
        """
        if not self.clean_task_running:
            return

        # ������䵼����״̬
        self.navigation("2")
        self.navigation_even.wait() # �ȴ��������
        self.navigation_even.clear() # �����������¼�
        # Socketͨ�Ŵ���
        self.socket_handle()
        if self.stop == True:
            self.ali_pub.publish(String("box_open"))
        # �ȴ����ӹر��¼�����
        self.wait_for_box_close_even.wait()
        self.wait_for_box_close_even.clear()
        self.showdistance = False # ��ʾ�����־λ��λ
        # ��ʼִ�е��˳���
        self.backward()
        # ���������
        self.navigation("1")
        self.clean_flag = True # ��ʾ�������
        self.navigation_even.wait() # �ȴ��������
        self.navigation_even.clear()

        # ����״̬���ظ�����ʼ������
        self.Init_state()

    def shutdown(self):
        """�ڵ��˳�ʱ��ȫ��ֹ�����̺߳���Դ���޸Ĳ��֣�"""
        rospy.loginfo("�ڵ�ر��У�������Դ...")
        self._shutdown_flag = True
        self.stop_sending()  # ����ֹͣ�����߳�
        self.stop_clean_task()
        # ǿ��ֹͣ�������˶�
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub.publish(self.vel_cmd)
        try:
            self.s.close()
        except:
            pass

if __name__ == '__main__':
    clean_task = CatPaw()
    rospy.on_shutdown(clean_task.shutdown)
    rospy.spin()