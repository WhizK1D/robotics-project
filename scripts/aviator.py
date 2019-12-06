#!/usr/bin/env python
from std_msgs.msg import String, Int8
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
walk.init_x_offset = -0.00999999977648
walk.init_y_offset = 0.0599999986589
walk.init_z_offset = 0.0399999991059
walk.init_roll_offset = 0.0
walk.init_pitch_offset = 0.0
walk.init_yaw_offset = 0.0
walk.period_time = 1.000000023842
walk.dsp_ratio = 0.20000000298
walk.step_fb_ratio = 0.40000000596
walk.x_move_amplitude = 0.2
walk.y_move_amplitude = 0.1
walk.z_move_amplitude = 0.015
walk.angle_move_amplitude = 0.0
walk.move_aim_on = False
walk.balance_enable = False
walk.balance_hip_roll_gain = 0.34999999404
walk.balance_knee_gain = 0.300000011921
walk.balance_ankle_roll_gain = 0.699999988079
walk.balance_ankle_pitch_gain = 0.899999976158
walk.y_swap_amplitude = 0.0349999996647
walk.z_swap_amplitude = 0.00999999977648
walk.arm_swing_gain = 1.0
walk.pelvis_offset = 0.0349065847695
walk.hip_pitch_offset = 0.122173048556
walk.p_gain = 0
walk.i_gain = 0
walk.d_gain = 0
# walk.x_move_amplitude = 0.020
# walk.y_move_amplitude = 0.020
# walk.z_move_amplitude = 0.020
# walk.period_time = 1000 * 0.001
walking_module = False
def get_base_truth(bot_data):
    print(bot_data)
    if bot_data.data == "walking_module":
        op3_walk_param = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=10)
        # while op3_walk_param.get_num_connections() <= 1:
        #     if op3_walk_param.get_num_connections() == 1:
        #         op3_walk_param.publish(walk)
        #         break        sleep(5)
        op3_walk_control = rospy.Publisher('/robotis/walking/command', String, queue_size=10)
        #while op3_walk_control.get_num_connections() <= 1:
        #    print(op3_walk_control.get_num_connections())
        #    if op3_walk_control.get_num_connections() == 1:
        #        print("walking module publish")
        #        #op3_walk_param.publish(walk)
        #        op3_walk_control.publish("start")
        #        break
        #sleep(10)
        # op3_walk_control = rospy.Publisher('/robotis/walking/command', String, queue_size=10)
        op3_walk_control.publish("stop")
        sleep(5)
        walk.x_move_amplitude = 0.000
        walk.y_move_amplitude = 0.000
        walk.angle_move_amplitude = - 0.125
        # op3_walk_control = rospy.Publisher('/robotis/walking/command', String, queue_size=10)
        while op3_walk_control.get_num_connections() <= 1:
            print(op3_walk_control.get_num_connections())
            if op3_walk_control.get_num_connections() == 1:
                print("walking module publish")
                op3_walk_param.publish(walk)
                op3_walk_control.publish("start")
                break
        sleep(10)
        # op3_walk_control = rospy.Publisher('/robotis/walking/command', String, queue_size=10)
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

def key_binding(key):
    op3_walk_param = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=10)
    op3_walk_control = rospy.Publisher('/robotis/walking/command', String, queue_size=10)
    print(key)
    if key.data == 65:
        # call some function here
        print("65 pressed")
        op3_walk_control.publish("stop")
        walk.x_move_amplitude = 0.020
        walk.y_move_amplitude = 0.010
        walk.angle_move_amplitude = 0.000
        op3_walk_param.publish(walk)
        op3_walk_control.publish("start")
    elif key.data == 66:
        print("66 pressed")
        op3_walk_control.publish("stop")
        #walk.x_move_amplitude = 0.000
        #walk.y_move_amplitude = 0.000
        #walk.angle_move_amplitude = 0.000
    elif key.data == 67:
        print("67 pressed")
        op3_walk_control.publish("stop")
        walk.x_move_amplitude = 0.000
        walk.y_move_amplitude = 0.000
        walk.angle_move_amplitude = - 0.125
        op3_walk_param.publish(walk)
        op3_walk_control.publish("start")
    elif key.data == 68:
        print("68 pressed")
        op3_walk_control.publish("stop")
        walk.x_move_amplitude = 0.000
        walk.y_move_amplitude = 0.000
        walk.angle_move_amplitude = 0.125
        op3_walk_param.publish(walk)
        op3_walk_control.publish("start")
    rospy.spin

def init():
    rospy.init_node("aviator", anonymous=False)
    rate = rospy.Rate(10)
    rospy.Subscriber('/robotis/base/ini_pose', String, get_base_truth)
    rospy.Subscriber('/robotis/enable_ctrl_module', String, get_base_truth)
    rospy.Subscriber('/robotis/present_joint_ctrl_modules', String, get_base_truth)
    rospy.Subscriber('/robotis/status', StatusMsg, init_pose_complete)

    rospy.Subscriber('/key', Int8, key_binding)
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
