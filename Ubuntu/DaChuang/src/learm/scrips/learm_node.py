#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import serial
import threading
import time

# 全局变量
ser = None
pub = None
topic_data = "None"
initial_state = bytearray([0x55, 0x55, 0x05, 0x06, 0x00, 0x01, 0x00]) # 0号动作组，机械臂的初始位置
state_one  = bytearray([0x55, 0x55, 0x05, 0x06, 0x01, 0x01, 0x00]) # 1号动作组，机械臂拿取
state_two = bytearray([0x55, 0x55, 0x05, 0x06, 0x02, 0x01, 0x00]) # 2号动作组， 机械臂倾倒
state_three = bytearray([0x55, 0x55, 0x05, 0x06, 0x03, 0x01, 0x00]) # 3号动作组， 机械臂放回
event_topic_done = threading.Event()
event_ack_received = threading.Event()  # 用于接收确认信号
event_action_done = threading.Event()   # 用于接收完成信号
max_retries = 3 # 最大尝试次数
current_retries = 0 # 当前尝试的次数
thread_running = True
learm_thread = None

def pub_finish():
    """
    当机械臂动作完成之后发布
    """
    learm_msg = String()
    learm_msg.data = "done"
    pub.publish(learm_msg)

def learm_callback(msg):
    """
    接收wp_node发送过来的动作指令
    """
    global topic_data
    topic_data = msg.data
    rospy.loginfo("topic data received %s", topic_data)
    event_topic_done.set()

def process_frame(frame):
    """
    处理接收到的数据帧
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
    """持续读取串口数据"""
    buffer = bytearray()
    while not rospy.is_shutdown():
        if ser and ser.isOpen():
            try:
                # 使用缓冲区处理粘包问题
                data = ser.read(ser.in_waiting or 1)
                buffer.extend(data)
                
                # 查找完整帧（55 55开头）
                while len(buffer) >= 4:
                    # 查找帧头
                    found = False
                    for i in range(len(buffer)-3):
                        if buffer[i] == 0x55 and buffer[i+1] == 0x55:
                            # 提取4字节帧
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
    """机械臂控制线程"""
    global topic_data, current_retries
    while thread_running:
        event_topic_done.wait()
        event_topic_done.clear()
        
        current_retries = 0
        cmd_sent = False
        
        while current_retries < max_retries and not cmd_sent:
            # 发送控制指令
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
            
            # 等待确认（2秒超时）
            event_ack_received.clear()
            if event_ack_received.wait(2.0):
                rospy.loginfo("OK")
                cmd_sent = True
                
                # 等待动作完成（5秒超时）
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
    
    # 初始化串口
    port = rospy.get_param('~port', '/dev/ttyUSB_Basedriver')
    baudrate = rospy.get_param('~baudrate', 9600)
    
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        ser.write(initial_state)  # 发送初始化指令
        rospy.loginfo("serial init succeed")
    except Exception as e:
        rospy.logerr("serial init failed: %s", str(e))
        return
    
    # 创建控制线程
    learm_thread = threading.Thread(target=learm_threadfunc)
    learm_thread.start()
    
    # 初始化发布/订阅
    pub = rospy.Publisher('learm_result', String, queue_size=10)
    rospy.Subscriber('/learm_control', String, learm_callback, queue_size=10)
    
    # 启动串口读取
    read_serial(pub)

if __name__ == '__main__':
    main()