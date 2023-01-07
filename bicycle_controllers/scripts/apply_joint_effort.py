#!/usr/bin/env python

import roslib
import rospy
import time
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

if __name__ == '__main__':
  rospy.init_node('apply_joint_effort_test')

  rospy.wait_for_service('/gazebo/apply_joint_effort')

  #time.sleep(10);
  apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
  joint_name = "back_wheel_joint"
  effort = -30
  start_time = rospy.Duration.from_sec(0)
  duration = rospy.Duration.from_sec(-1)

  try:
    resp1 = apply_joint_effort(joint_name, effort, start_time, duration)
    print(resp1)
  except rospy.ServiceException as e:
    print("Service did not process request: %s"%str(e))



  #apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
  #joint_name = "back_wheel_joint"
  effort = 30
  start_time = rospy.Duration.from_sec(3)
  duration = rospy.Duration.from_sec(-1)

  try:
    resp1 = apply_joint_effort(joint_name, effort, start_time, duration)
    print(resp1)
  except rospy.ServiceException as e:
    print("Service did not process request: %s"%str(e))