#!/usr/bin/env python
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

import rosbag
import rospy
import numpy
import math
import yaml
from time import sleep

from robotis_controller_msgs.msg import StatusMsg
from op3_walking_module_msgs.msg import WalkingParam
from robotis_controller_msgs.srv import GetJointModule

walk = WalkingParam()
walk.x_move_amplitude = 0.020
walk.y_move_amplitude = 0.020
walk.z_move_amplitude = 0.020
walk.period_time = 1000 * 0.001

walking_module = False


def get_base_truth(bot_data):
    print(bot_data)
    if bot_data.data == "walking_module":
        # op3_walk_param = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=10)
        # while op3_walk_param.get_num_connections() <= 1:
        #     if op3_walk_param.get_num_connections() == 1:
        #         op3_walk_param.publish(walk)
        #         break

        sleep(5)
        op3_walk_control = rospy.Publisher('/robotis/walking/command', String, queue_size=10)
        while op3_walk_control.get_num_connections() <= 1:
            print(op3_walk_control.get_num_connections())
            if op3_walk_control.get_num_connections() == 1:
                print("walking module publish")
                op3_walk_control.publish("start")
                break

        sleep(10)
        op3_walk_control = rospy.Publisher('/robotis/walking/command', String, queue_size=10)
        op3_walk_control.publish("stop")


def init_pose_complete(msg):
    print (msg.status_msg)
    global walking_module
    if msg.status_msg == "Finish Init Pose":
        op3_walk = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10)
        while op3_walk.get_num_connections() <= 2:
            if op3_walk.get_num_connections() == 2:
                op3_walk.publish("walking_module")
                break
        # get_current_joint_status()


def init():
    rospy.init_node("aviator", anonymous=False)
    rate = rospy.Rate(10)
    rospy.Subscriber('/robotis/base/ini_pose', String, get_base_truth)
    rospy.Subscriber('/robotis/enable_ctrl_module', String, get_base_truth)
    rospy.Subscriber('/robotis/present_joint_ctrl_modules', String, get_base_truth)
    rospy.Subscriber('/robotis/status', StatusMsg, init_pose_complete)

    # get_current_joint_status()

    op3 = rospy.Publisher('/robotis/base/ini_pose', String, queue_size=10)

    while op3.get_num_connections() <= 2:
        if op3.get_num_connections() == 2:
            op3.publish("ini_pose")
            break
        rate.sleep()

    rospy.spin()


def get_current_joint_status():
    rospy.wait_for_service('/robotis/get_present_joint_ctrl_modules')
    try:
        status = rospy.ServiceProxy('joint_name', GetJointModule)
        print (status)
        status.call("joint_name")
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException:
        raise Exception("ROSInterruptException")
        pass
