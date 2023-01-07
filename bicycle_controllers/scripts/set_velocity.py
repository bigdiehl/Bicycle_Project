#!/usr/bin/env python3

# Test program to send a velocity command to the rear bicycle wheel. Will 
# set velocity to current value of parameter initial_conditions/vel

import roslib
import rospy
import time

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

if __name__ == '__main__':
    pub = rospy.Publisher('bicycle/vel_cmd', JointState, queue_size=10)
    #pub = rospy.Publisher('bicycle/torque_cmd', JointState, queue_size=10)

    rospy.init_node('velocity_commander')
    rate = rospy.Rate(1) # 1hz

    #vel = rospy.get_param("initial_conditions/vel", 2);
    vel=10

    while not rospy.is_shutdown():
        try:
            log_str = "Setting vel to " + str(vel)
            rospy.loginfo(log_str)

            vel_cmd = JointState()
            vel_cmd.header = Header()
            vel_cmd.name = ["back_wheel_joint"]
            vel_cmd.velocity = [vel]
            vel_cmd.position = []
            vel_cmd.effort = []

            pub.publish(vel_cmd)
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")

    




