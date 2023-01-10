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

from geometry_msgs.msg import Wrench, Point, Vector3


if __name__ == '__main__':

    rospy.init_node('bicycle_dist_gen')

    model_name = 'bicycle'

    parser = argparse.ArgumentParser(description="bicycle disturbance generator")
    parser.add_argument('duration', help="Duration in sec")
    parser.add_argument('-f', dest="force", default='[0,0,0]', help="Disturbance force. Expects list of [x,y,z]")
    parser.add_argument('-t', dest="torque", default='[0,0,0]', help="Disturbance torque. Expects list of [x,y,z]")
    args = parser.parse_args()

    
    print(args)

    rospy.wait_for_service('/gazebo/apply_joint_effort')
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    rospy.wait_for_service('/gazebo/clear_body_wrenches')
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    rospy.wait_for_service('/gazebo/set_joint_properties')
    rospy.wait_for_service('/gazebo/set_link_properties')
    rospy.wait_for_service('/gazebo/get_model_properties')

    model_props = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    joint_props = rospy.ServiceProxy('/gazebo/set_joint_properties', GetJointProperties)
    link_props = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)

    # ['body_name', 'reference_frame', 'reference_point', 'wrench', 'start_time', 'duration']
    wrench_srv = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    # Get nominal model, joint, and link states
    model = model_props(model_name)
    #print(model)

    back_frame = link_props('back_frame')
    
    wrench = Wrench()
    wrench.force = Vector3(*eval(args.force))
    wrench.torque = Vector3(*eval(args.torque))

    success = wrench_srv("bicycle::back_frame",
        "bicycle::back_frame",
        back_frame.com.position,
        wrench,
        rospy.Time().now(), 
        rospy.Duration(float(args.duration)))

    print(success)
    







