#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

class QRCodeFollower:
    def __init__(self):
        rospy.init_node('qr_code_follower')
        
        # ���Ķ�ά��λ�û���
        self.sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.qr_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # ���Ʋ���
        self.target_threshold = 0.03    # ֹͣ��ֵ����λ���ף�����0.02~0.05��
        self.fixed_angular_speed = 0.05 # �̶����ٶȣ�rad/s��

    def qr_callback(self, msg):
        twist = Twist()
        if len(msg.markers) > 0:
            qr_pose = msg.markers[0].pose.pose.position
            
            # ����ˮƽƫ�����ͷ����ϵ��
            error_x = qr_pose.x
            
            if abs(error_x) > self.target_threshold:
                # ����ƫ���ѡ����ת����
                if error_x > 0:  # ��ά�����Ҳ࣬��Ҫ˳ʱ����ת��angular.zΪ����
                    twist.angular.z = -self.fixed_angular_speed
                else:            # ��ά������࣬��Ҫ��ʱ����ת��angular.zΪ����
                    twist.angular.z = self.fixed_angular_speed
                rospy.loginfo(f"Adjusting: error_x={error_x:.3f}m, speed={twist.angular.z:.2f}rad/s")
            else:
                rospy.loginfo("QR Code centered! Stopping...")
        else:
            twist.angular.z = 0  # δ��⵽��ά��ʱֹͣ
            
        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        follower = QRCodeFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass