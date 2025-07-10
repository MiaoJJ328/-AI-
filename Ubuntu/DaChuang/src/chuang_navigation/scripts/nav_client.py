#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__ == "__main__":
    rospy.init_node("nav_client")

    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    ac.wait_for_server()

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = -0.998802
    goal.target_pose.pose.position.y = 1.87808
    goal.target_pose.pose.position.z = 0

    goal.target_pose.pose.orientation.x = 0 
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = -0.708132
    goal.target_pose.pose.orientation.w = 0.70608

    ac.send_goal(goal)

    rospy.loginfo("begin...")

    ac.wait_for_result()

    if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("succeed")
    else:
        rospy.loginfo("failed...")


