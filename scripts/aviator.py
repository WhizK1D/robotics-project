#!/usr/bin/env python
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

import rosbag
import rospy
import numpy
import math
import yaml

from robotis_controller_msgs.msg import StatusMsg


def get_base_truth(bot_data):
    print(bot_data)


def init_pose_complete(msg):
    print (msg.status_msg)
    if msg.status_msg == "Finish Init Pose":
        op3_walk = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10)
        while op3_walk.get_num_connections() <= 2:
            if op3_walk.get_num_connections() == 2:
                op3_walk.publish("walking_module")
                break


def init():
    rospy.init_node("aviator", anonymous=False)
    rate = rospy.Rate(10)
    rospy.Subscriber('/robotis/base/ini_pose', String, get_base_truth)
    rospy.Subscriber('/robotis/enable_ctrl_module', String, get_base_truth)
    # rospy.Subscriber('/robotis/present_joint_ctrl_modules', String, get_base_truth)
    rospy.Subscriber('/robotis/status', StatusMsg, init_pose_complete)

    op3 = rospy.Publisher('/robotis/base/ini_pose', String, queue_size=10)

    while op3.get_num_connections() <= 2:
        if op3.get_num_connections() == 2:
            op3.publish("ini_pose")
            break
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException:
        raise Exception("ROSInterruptException")
        pass
