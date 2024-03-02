#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import WrenchStamped
import numpy as np
import os

class Step1Recorder:
    def __init__(self) -> None:
        rospy.init_node('/step1/recorder', anonymous=True)

        # subscribers 
        self.force_torque_sub = \
            rospy.Subscriber('/force_torque_sensor_controller', \
                             WrenchStamped, self.force_torque_callback) #TODO: check the topic name

        # pressing
        self.pressing_client = rospy.ServiceProxy('/step1/pressing/stop', Empty)
        rospy.Service('/step1/pressing/start', Empty, self.pressing_server)
        self.pressing_recording = False

        # lateral movement right
        self.lateral_movement_right_client = rospy.ServiceProxy('/step1/lateral_movement_right/stop', Empty)
        rospy.Service('/step1/lateral_movement_right/start', Empty, self.lateral_movement_right_server)
        self.lateral_movement_right_recording = False

        # lateral movement left
        self.lateral_movement_left_client = rospy.ServiceProxy('/step1/lateral_movement_left/stop', Empty)
        rospy.Service('/step1/lateral_movement_left/start', Empty, self.lateral_movement_left_server)
        self.lateral_movement_left_recording = False

        self.save_data_name = input('Enter the save file name: ')
        self.save_data_dir = '/root/Research_Internship_at_GVlab/real_exp/step1/'
        self.pressing_data = None
        self.lateral_movement_right_data = None
        self.lateral_movement_left_data = None
        self.data = None

    def pressing_server(self, req):
        self.pressing_recording = True

    def lateral_movement_right_server(self, req):
        self.lateral_movement_right_recording = True

    def lateral_movement_left_server(self, req):
        self.lateral_movement_left_recording = True

    def force_torque_callback(self, msg):
        if self.pressing_recording:
            while not rospy.is_shutdown():
                print('pressing', msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z)
                if self.pressing_data is None:
                    self.pressing_data = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
                else:
                    self.pressing_data = np.vstack((self.pressing_data, [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])) #(n, 6)
                # 200個データが集まったら抜ける
                if self.pressing_data.shape[0] == 200:
                    self.pressing_data = np.expand_dims(self.pressing_data, axis=0) #(1, 200, 6)
                    self.pressing_client()
                    self.pressing_recording = False
                    self.data = self.pressing_data #(1, 200, 6)
                    break

        elif self.lateral_movement_right_recording:
            while not rospy.is_shutdown():
                print('lateral_movement_right', msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z)
                if self.lateral_movement_right_data is None:
                    self.lateral_movement_right_data = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
                else:
                    self.lateral_movement_right_data = np.vstack((self.lateral_movement_right_data, [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]))
                # 100個データが集まったら抜ける
                if self.lateral_movement_right_data.shape[0] == 100: 
                    self.lateral_movement_right_client()
                    self.lateral_movement_right_recording = False
                    break

        elif self.lateral_movement_left_recording:
            while not rospy.is_shutdown():
                print('lateral_movement_left', msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z)
                if self.lateral_movement_left_data is None:
                    self.lateral_movement_left_data = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
                else:
                    self.lateral_movement_left_data = np.vstack((self.lateral_movement_left_data, [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]))
                # 100個データが集まったら抜ける
                if self.lateral_movement_left_data.shape[0] == 100:
                    self.lateral_movement_left_client()
                    self.lateral_movement_left_recording = False
                    lateral_movement_data = np.concatenate([self.lateral_movement_right_data, self.lateral_movement_left_data], axis=0) #(200, 6)
                    lateral_movement_data = np.expand_dims(lateral_movement_data, axis=0) #(1, 200, 6)
                    self.data = np.concatenate([self.pressing_data, lateral_movement_data], axis=0) #(2, 200, 6)
                    if not os.path.exists(self.save_data_dir):
                        os.makedirs(self.save_data_dir)
                    np.save(self.save_data_dir + self.save_data_name, self.data)
                    rospy.loginfo('Data saved')
                    break

if __name__ == '__main__':
    step1_recorder = Step1Recorder()
    rospy.spin()

