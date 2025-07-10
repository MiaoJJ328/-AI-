#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import json
from std_msgs.msg import String
class WaypointNavigator:
    def __init__(self):
        rospy.init_node("wp_node")
        
        # 航点位置描述（使用类变量而不是全局变量）
        self.location_names = {
            1: "客厅",
            2: "厨房", 
            3: "卧室",
            4: "大门",
            5: "浴室"
        }
        self.current_destination = None  # 当前目标航点
        
        # 初始化发布器和订阅器
        self.navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10)
        self.res_sub = rospy.Subscriber("/waterplus/navi_result", String, self.nav_result_callback, queue_size=10)
        # 订阅阿里云消息
        self.ali_sub = rospy.Subscriber("/paw_control", String, self.aliyunCallback, queue_size=10)
        # 用于向阿里云发布消息
        self.ali_pub = rospy.Publisher("/sparrow_nav", String, queue_size=10)
        
        rospy.loginfo("Waypoint Navigator 节点已启动，等待指令...")
        
        rospy.sleep(1)  # 等待发布者和订阅者建立连接
        
    def aliyunCallback(self, msg):
        """
        处理从/car_control接收的原始JSON数据
        消息格式示例：'{"nav_location":2}' （直接是JSON字符串）
        """
        rospy.loginfo("回调函数被触发，原始消息: %s", msg.data)
      
        try:
            # 直接解析JSON字符串
            data = json.loads(msg.data)
            rospy.loginfo("解析后的JSON数据: %s", data)
            
            # 检查必要字段
            if "nav_location" not in data:
                rospy.logwarn("JSON缺少nav_location字段")
                return
                
            value = data["nav_location"]
            
            # 验证航点有效性
            if value not in self.location_names:
                rospy.logwarn("收到无效航点ID: %s", value)
                return
                
            # 执行导航
            self.current_destination = value
            self.navigate_to_waypoint(str(value))
            rospy.loginfo("正在导航至: %s", self.location_names[value])
            
            # 可选：发送反馈
            # feedback = {
            #     "status": "navigating",
            #     "target": value,
            #     "timestamp": rospy.get_time()
            # }
            # self.ali_pub.publish(String(json.dumps(feedback)))
            
        except json.JSONDecodeError as e:
            rospy.logerr("JSON解析失败！原始数据: %s, 错误: %s", msg.data, str(e))
        except Exception as e:
            rospy.logerr("处理消息时发生意外错误: %s", str(e))
            
        
    def nav_result_callback(self, msg):
        """
        导航结果回调函数
        """
        if self.current_destination:
            location_name = self.location_names.get(self.current_destination, "未知位置")
            rospy.logwarn("导航结果: {} -> {}".format(msg.data, location_name))
            
        
    def navigate_to_waypoint(self, waypoint):
        """
        发布导航到指定航点的指令
        """
        navi_msg = String()
        navi_msg.data = str(waypoint)
        self.navi_pub.publish(navi_msg)
        rospy.loginfo("正在导航至: {}号航点 ({})".format(
            waypoint, 
            self.location_names.get(int(waypoint), "未知位置")
        ))
        
    def run(self):
        """
        运行节点
        """
        rospy.spin()

if __name__ == "__main__":
    navigator = WaypointNavigator()
    navigator.run()