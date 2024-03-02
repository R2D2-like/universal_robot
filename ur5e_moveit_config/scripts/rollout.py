#!/usr/bin/env python3

import argparse
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

import signal

DATA_PATH = '/root/Research_Internship_at_GVlab/real_exp/rollout/rollout_data.npy'

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class Rollout:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('/rollout', anonymous=True)

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher

        rospy.loginfo('Rollout node initialized')

    def rollout(self):
        rollout_traj = np.load(DATA_PATH) #(2000, 3)
        rate = rospy.Rate(10)
        for i in range(rollout_traj.shape[0]):
            pose_goal = self.move_group.get_current_pose().pose
            pose_goal.position.x = rollout_traj[i, 0]
            pose_goal.position.y = rollout_traj[i, 1]
            pose_goal.position.z = rollout_traj[i, 2]
            self.move_group.set_pose_target(pose_goal)
            plan = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rate.sleep()
        rospy.signal_shutdown('done')

if __name__ == '__main__':
    rollout = Rollout()
    rollout.rollout()
    rospy.spin()
    
