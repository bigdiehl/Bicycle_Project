#!/usr/bin/env python3

""" Utility to command the bicycle. Can run continuously or issue a single command 
depending on the arguments 

--sine (specify frequency)
--ramp (specify rate) (default is step)

--continuous
--speed 
--steer_angle
--heading 
--roll 
""" 

import roslib
import rospy
import time
from bicycle_msgs.msg import BicycleCmd
from std_msgs.msg import Header
import sys
import argparse
import numpy as np

if __name__ == '__main__':

  rospy.init_node('bicycle_commander')
  d0 = rospy.get_param("bicycle/initial_conditions/delta", 0)
  v0 = rospy.get_param("bicycle/initial_conditions/vel", 0)

  parser = argparse.ArgumentParser(description="bicycle_commander node. Default is step, but can command ramp or sine instead")
  parser.add_argument('-v', '--speed', dest="speed", default=v0, help="Desired speed (v) in m/s")
  parser.add_argument('-d', '--steer_angle', dest="delta", default=d0, help="Desired steer_angle (d) in deg")
  parser.add_argument('-p', '--heading', dest="heading", default=0, help="Desired heading (p) in deg")

  # TODO - implement these
  parser.add_argument('-r', '--ramp', dest="ramp", default=0, help="Implement ramp for [v,d,p]. Provide ramp slope.")
  parser.add_argument('-s', '--sine', dest="sine", default=0, help="Implement sine for [v,d,p]. Provide frequency (Hz).")

  args = parser.parse_args()
  print(args)

  command_pub = rospy.Publisher("bicycle/command", BicycleCmd, queue_size=1)

  cmd = BicycleCmd()
  cmd.desired_steer_angle = np.radians(float(args.delta))
  cmd.desired_speed = float(args.speed)

  rate = rospy.Rate(10) # Hz

  while not rospy.is_shutdown():
    try:
      h = Header()
      h.stamp = rospy.Time.now()
      cmd.header = h
      command_pub.publish(cmd)
      rate.sleep()
    except rospy.ROSTimeMovedBackwardsException:
      rospy.logerr("ROS Time Backwards! Just ignore the exception!")