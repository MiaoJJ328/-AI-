#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import serial
import threading
import time

# ȫ�ֱ���
ser = None
pub = None
topic_data = "None"
initial_state = bytearray([0x55, 0x55, 0x05, 0x06, 0x00, 0x01, 0x00]) # 0�Ŷ����飬��е�۵ĳ�ʼλ��
state_one  = bytearray([0x55, 0x55, 0x05, 0x06, 0x01, 0x01, 0x00]) # 1�Ŷ����飬��е����ȡ
state_two = bytearray([0x55, 0x55, 0x05, 0x06, 0x02, 0x01, 0x00]) # 2�Ŷ����飬 ��е���㵹
state_three = bytearray([0x55, 0x55, 0x05, 0x06, 0x03, 0x01, 0x00]) # 3�Ŷ����飬 ��е�۷Ż�
event_topic_done = threading.Event()
event_ack_received = threading.Event()  # ���ڽ���ȷ���ź�
event_action_done = threading.Event()   # ���ڽ�������ź�
max_retries = 3 # ����Դ���
current_retries = 0 # ��ǰ���ԵĴ���
thread_running = True
learm_thread = None

def pub_finish():
    """
    ����е�۶������֮�󷢲�
    """
    learm_msg = String()
    learm_msg.data = "done"
    pub.publish(learm_msg)

def learm_callback(msg):
    """
    ����wp_node���͹����Ķ���ָ��
    """
    global topic_data
    topic_data = msg.data
    rospy.loginfo("topic data received %s", topic_data)
    event_topic_done.set()

def process_frame(frame):
    """
    ������յ�������֡
    """
    if len(frame) >= 4:
        hex_bytes = [frame[0], frame[1], frame[2], frame[3]]
        # rospy.loginfo("Received: 0x%02X 0x%02X 0x%02X 0x%02X", *hex_bytes)
        
        if hex_bytes[0] == 0x55 and hex_bytes[1] == 0x55:
            if hex_bytes[2] == 0x02 and hex_bytes[3] == 0x06:
                rospy.loginfo("received command")
                event_ack_received.set()
            elif hex_bytes[2] == 0x02 and hex_bytes[3] == 0x07:
                rospy.loginfo("action finished")
                pub_finish()
                event_action_done.set()

def read_serial(pub):
    """������ȡ��������"""
    buffer = bytearray()
    while not rospy.is_shutdown():
        if ser and ser.isOpen():
            try:
                # ʹ�û���������ճ������
                data = ser.read(ser.in_waiting or 1)
                buffer.extend(data)
                
                # ��������֡��55 55��ͷ��
                while len(buffer) >= 4:
                    # ����֡ͷ
                    found = False
                    for i in range(len(buffer)-3):
                        if buffer[i] == 0x55 and buffer[i+1] == 0x55:
                            # ��ȡ4�ֽ�֡
                            frame = buffer[i:i+4]
                            buffer = buffer[i+4:]
                            process_frame(frame)
                            found = True
                            break
                    if not found:
                        break

            except Exception as e:
                rospy.logerr("Read error: %s", str(e))
        time.sleep(0.01)

def learm_threadfunc():
    """��е�ۿ����߳�"""
    global topic_data, current_retries
    while thread_running:
        event_topic_done.wait()
        event_topic_done.clear()
        
        current_retries = 0
        cmd_sent = False
        
        while current_retries < max_retries and not cmd_sent:
            # ���Ϳ���ָ��
            if topic_data == "pickup":
                ser.write(state_one)
                rospy.loginfo("send pickup...")
            elif topic_data == "pour":
                ser.write(state_two)
                rospy.loginfo("send pour...")
            elif topic_data == "setdown":
                ser.write(state_three)
                rospy.loginfo("send setdown...")
            elif topic_data == "init_pos":
                ser.write(initial_state)
                rospy.loginfo("send init...")
            
            # �ȴ�ȷ�ϣ�2�볬ʱ��
            event_ack_received.clear()
            if event_ack_received.wait(2.0):
                rospy.loginfo("OK")
                cmd_sent = True
                
                # �ȴ�������ɣ�5�볬ʱ��
                event_action_done.clear()
                if event_action_done.wait(10.0):
                    rospy.loginfo("action done")
                else:
                    rospy.logwarn("action failed")
            else:
                rospy.logwarn("current_retries%d...", current_retries+1)
                current_retries += 1

        if not cmd_sent:
            rospy.logerr("current_retries got max")
        
        topic_data = "None"

def cleanup():
    global thread_running, learm_thread
    thread_running = False
    event_topic_done.set()
    
    if ser and ser.isOpen():
        ser.close()
        rospy.loginfo("serial shutdoown")
    
    if learm_thread is not None:
        learm_thread.join(timeout=1.0)
        rospy.loginfo("thread out")

def main():
    global ser, pub, learm_thread
    rospy.init_node('learm_node', anonymous=False)
    rospy.on_shutdown(cleanup)
    
    # ��ʼ������
    port = rospy.get_param('~port', '/dev/ttyUSB_Basedriver')
    baudrate = rospy.get_param('~baudrate', 9600)
    
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        ser.write(initial_state)  # ���ͳ�ʼ��ָ��
        rospy.loginfo("serial init succeed")
    except Exception as e:
        rospy.logerr("serial init failed: %s", str(e))
        return
    
    # ���������߳�
    learm_thread = threading.Thread(target=learm_threadfunc)
    learm_thread.start()
    
    # ��ʼ������/����
    pub = rospy.Publisher('learm_result', String, queue_size=10)
    rospy.Subscriber('/learm_control', String, learm_callback, queue_size=10)
    
    # �������ڶ�ȡ
    read_serial(pub)

if __name__ == '__main__':
    main()