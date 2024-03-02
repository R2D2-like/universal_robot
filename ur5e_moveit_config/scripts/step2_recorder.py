#!/usr/bin/env python3
import rospy
import numpy as np
import os
import tf

class Step2Recorder:
    def __init__(self) -> None:
        rospy.init_node('/step2/recorder', anonymous=True)

        # tf listener
        self.listener = tf.TransformListener()
        self.save_data_name = input('Enter the save file name: ')
        self.save_data_dir = '/root/Research_Internship_at_GVlab/real_exp/step2/'
        self.data = None
        rospy.loginfo('Initialized Step2Recorder')

    def record(self):
        input('Press Enter to start recording')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                pos, rot = self.listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
                if self.data is None:
                    # only record the position
                    self.data = np.array(pos)
                else:
                    self.data = np.vstack((self.data, pos)) #(n, 3)
                if self.data.shape[0] == 2000:
                    if not os.path.exists(self.save_data_dir):
                        os.makedirs(self.save_data_dir)
                    np.save(os.path.join(self.save_data_dir, self.save_data_name), self.data)
                    rospy.signal_shutdown('done')
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()

if __name__ == '__main__':
    step2_recorder = Step2Recorder()
    step2_recorder.record()
    rospy.spin()

