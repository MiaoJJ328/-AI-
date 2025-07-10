#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

class QRCodeFollower:
    def __init__(self):
        rospy.init_node('qr_code_follower')
        
        # 订阅二维码位置话题
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.qr_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 控制参数
        self.target_threshold = 0.03    # 停止阈值（单位：米，建议0.02~0.05）
        self.fixed_angular_speed = 0.05 # 固定角速度（rad/s）

    def qr_callback(self, msg):
        twist = Twist()
        if len(msg.markers) > 0:
            qr_pose = msg.markers[0].pose.pose.position
            
            # 计算水平偏差（摄像头坐标系）
            error_x = qr_pose.x
            
            if abs(error_x) > self.target_threshold:
                # 根据偏差方向选择旋转方向
                if error_x > 0:  # 二维码在右侧，需要顺时针旋转（angular.z为负）
                    twist.angular.z = -self.fixed_angular_speed
                else:            # 二维码在左侧，需要逆时针旋转（angular.z为正）
                    twist.angular.z = self.fixed_angular_speed
                rospy.loginfo(f"Adjusting: error_x={error_x:.3f}m, speed={twist.angular.z:.2f}rad/s")
            else:
                rospy.loginfo("QR Code centered! Stopping...")
        else:
            twist.angular.z = 0  # 未检测到二维码时停止
            
        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        follower = QRCodeFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass