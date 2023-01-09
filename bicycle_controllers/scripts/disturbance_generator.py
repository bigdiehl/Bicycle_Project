#!/usr/bin/env python3

""" Uses Gazebo service calls to generate disturbance force/torques on the Bicycle
joints/links  """


import rospy
import time
import sys
import argparse
import numpy as np
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *


if __name__ == '__main__':

    rospy.init_node('bicycle_dist_gen')

    parser = argparse.ArgumentParser(description="bicycle disturbance generator")

    args = parser.parse_args()


    rospy.wait_for_service('/gazebo/apply_joint_effort')
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    rospy.wait_for_service('/gazebo/clear_body_wrenches')
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    rospy.wait_for_service('/gazebo/set_joint_properties')
    rospy.wait_for_service('/gazebo/set_link_properties')
    rospy.wait_for_service('/gazebo/get_model_properties')


    




