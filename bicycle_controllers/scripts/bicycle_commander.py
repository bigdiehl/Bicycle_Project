#!/usr/bin/env python3

# Commands the bicycle

import roslib
import rospy
import time
from bicycle_msgs.msg import BicycleCmd
from std_msgs.msg import Header

if __name__ == '__main__':
  rospy.init_node('bicycle_commander')

  command_pub = rospy.Publisher("bicycle/command", BicycleCmd, queue_size=1)

  cmd = BicycleCmd()
  cmd.desired_steer_angle = 0
  cmd.desired_speed = 4

  rate = rospy.Rate(1) # Hz

  while not rospy.is_shutdown():

    try:
      h = Header()
      h.stamp = rospy.Time.now()
      command_pub.header = h
      command_pub.publish(cmd)
      rate.sleep()
    except rospy.ROSTimeMovedBackwardsException:
      rospy.logerr("ROS Time Backwards! Just ignore the exception!")