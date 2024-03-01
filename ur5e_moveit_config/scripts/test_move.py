#!/usr/bin/env python3
# coding: utf-8
 
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK

if __name__ == '__main__':
    # ROSノード初期化
    rospy.init_node('test_get_position_ik')
 
    # JointStateを取得
    joint_state = rospy.wait_for_message('joint_states', JointState)
    print(joint_state)
 
    # RobotStateを作成
    robot_state = RobotState()
    robot_state.joint_state = joint_state
 
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'base_link'
    pose_stamped.pose.position.x = 0.0
    pose_stamped.pose.position.y = -0.1
    pose_stamped.pose.position.z = -0.2
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0
 
    constraints = Constraints()
 
    try:
        request = PositionIKRequest()
        request.group_name = 'wrist_3'
        request.robot_state = robot_state
        request.constraints = constraints
        request.pose_stamped = pose_stamped
 
        compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        response = compute_ik(request)
        print(response)
 
    except rospy.ServiceException as e:
        print("Servie call failed: {}".format(e))