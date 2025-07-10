#!/usr/bin/env python
# coding: utf-8
import rospy
from geometry_msgs.msg import Twist

class TimedMotion:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('timed_motion_control')
        
        # 创建发布者
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 设置角速度参数
        self.angular_speed = 0.05  # 弧度/秒（根据机器人调整）
        self.total_duration = 10  # 总运行时间（秒）
        self.start_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(10)  # 10Hz控制频率
        
        while not rospy.is_shutdown():
            # 计算经过时间
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            
            # 创建速度指令
            cmd = Twist()
            
            if elapsed < 5:          # 0-5秒：左转
                cmd.angular.z = self.angular_speed
                rospy.loginfo("Turning LEFT | Elapsed: %.1fs" % elapsed)
            elif elapsed < 10:       # 5-10秒：右转
                cmd.angular.z = -self.angular_speed
                rospy.loginfo("Turning RIGHT | Elapsed: %.1fs" % elapsed)
            else:                    # 超过10秒停止
                rospy.loginfo("Motion completed")
                break
            
            # 发布指令
            self.cmd_pub.publish(cmd)
            rate.sleep()
        
        # 发送停止指令
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        controller = TimedMotion()
        controller.run()
    except rospy.ROSInterruptException:
        pass