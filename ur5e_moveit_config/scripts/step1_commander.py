#!/usr/bin/env python3
from std_msgs.msg import Float64MultiArray
import rospy
from std_srvs.srv import Empty

class Step1Commander:
    def __init__(self):
        rospy.init_node('/step1/commander', anonymous=True)
        self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        self.joint_vel = Float64MultiArray()

        #pressing
        rospy.Service('/step1/pressing/stop', Empty, self.pressing_server)
        self.stop_pressing = False
        self.pressing_client = rospy.ServiceProxy('/step1/pressing/start', Empty)

        #lateral movement right
        rospy.Service('/step1/lateral_movement_right/stop', Empty, self.lateral_movement_right_server)
        self.stop_lateral_movement_right = False
        self.lateral_movement_right_client = rospy.ServiceProxy('/step1/lateral_movement_right/start', Empty)

        #lateral movement left
        rospy.Service('/step1/lateral_movement_left/stop', Empty, self.lateral_movement_left_server)
        self.stop_lateral_movement_left = False
        self.lateral_movement_left_client = rospy.ServiceProxy('/step1/lateral_movement_left/start', Empty)
        
    def pressing_server(self, req):
        self.stop_pressing = True

    def pressing(self):
        #エンドエフェクタをｚ軸方向に速度0.01m/sでself.stop_pressingがTrueになるまで押す
        #['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_vel.data = [0, 0, 0.05, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)
        self.pressing_client()
        while not self.stop_pressing:
            rospy.sleep(0.1)
        self.joint_vel.data = [0, 0, 0, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)

    def lateral_movement_right_server(self, req):
        self.stop_lateral_movement_right = True

    def lateral_movement_right(self):
        #エンドエフェクタをｘ軸方向に速度0.05m/sでself.stop_lateral_movement_rightがTrueになるまで右に揺らす
        self.joint_vel.data = [0.05, 0, 0, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)
        self.lateral_movement_right_client()
        while not self.stop_lateral_movement_right:
            rospy.sleep(0.1)
        self.joint_vel.data = [0, 0, 0, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)

    def lateral_movement_left_server(self, req):
        self.stop_lateral_movement_left = True

    def lateral_movement_left(self):
        #エンドエフェクタをｘ軸方向に速度0.05m/sでself.stop_lateral_movement_leftがTrueになるまで左に揺らす
        self.joint_vel.data = [-0.05, 0, 0, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)
        self.lateral_movement_left_client()
        while not self.stop_lateral_movement_left:
            rospy.sleep(0.1)
        self.joint_vel.data = [0, 0, 0, 0, 0, 0]
        print(self.joint_vel)
        self.pub.publish(self.joint_vel)

    def main(self):
        self.pressing()
        self.lateral_movement_right()
        self.lateral_movement_left()
        print('done!')
        

if __name__ == '__main__':
    step1_commander = Step1Commander()
    step1_commander.main()