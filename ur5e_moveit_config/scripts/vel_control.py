#!/usr/bin/env python3
from std_msgs.msg import Float64MultiArray
import rospy

class ExpAction:
    def __init__(self):
        self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        self.joint_vel = Float64MultiArray()
        self.joint_vel.data = [0, -5, 10, -5, -5, 0]
        self.pub.publish(self.joint_vel)
        rospy.sleep(5)  
        self.joint_vel.data = [0, 0, 0, 0, 0, 0]
        rospy.sleep(1)

    def pressing(self):
        #エンドエフェクタをｚ軸方向に速度0.01m/sで2秒間押す
        #['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_vel.data = [0, 0, 0.05, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)
        rospy.sleep(5)
        self.joint_vel.data = [0, 0, 0, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)

    def lateral_movements(self):
        #エンドエフェクタをｘ軸方向に速度0.05m/sで1秒間左右に揺らす
        self.joint_vel.data = [0.05, 0, 0, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)
        rospy.sleep(5)
        self.joint_vel.data = [-0.05, 0, 0, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)
        rospy.sleep(5)
        self.joint_vel.data = [0, 0, 0, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)

if __name__ == '__main__':
    rospy.init_node('exp_action', anonymous=True)
    exp_action = ExpAction()
    exp_action.pressing()
    exp_action.lateral_movements()
    print('done!')