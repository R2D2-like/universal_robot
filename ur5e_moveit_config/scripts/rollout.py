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

    def plan_cartesian_path(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        ##
        # Cartesian Paths
        # ^^^^^^^^^^^^^^^
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through. If executing  interactively in a
        # Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        rollout_traj = np.load(DATA_PATH) #(2000, 3)
        for i in range(rollout_traj.shape[0]):
            wpose.position.x = rollout_traj[i, 0]
            wpose.position.y = rollout_traj[i, 1]
            wpose.position.z = rollout_traj[i, 2]
            waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def rollout(self):
        plan = self.plan_cartesian_path()
        self.move_group.execute(plan, wait=True)
        rospy.signal_shutdown('done')

if __name__ == '__main__':
    rollout = Rollout()
    rollout.rollout()
    rospy.spin()
    
