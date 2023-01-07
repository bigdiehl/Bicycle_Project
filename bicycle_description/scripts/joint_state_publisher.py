#!/usr/bin/env python2.7

import rospy
import time
from sensor_msgs.msg import JointState
from bicycle_msgs.msg import gyro_state
import numpy as np

class JointStatePublisher():

	def __init__(self):
		self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size = 1)
		self.whirlybird_sub = rospy.Subscriber('/gyro_states', gyro_state, self.gyro_stand_callback)
		
		time.sleep(1)

		state = JointState()
		state.header.stamp = rospy.Time.now()
		state.name = ['base_joint', 'gimbal_joint', 'flywheel_joint']
		state.position = [0, np.pi/2, 0]
		self.joint_state_pub.publish(state)

		rospy.spin()

	def gyro_stand_callback(self, msg):
		state = JointState()
		state.header.stamp = rospy.Time.now()
		state.name = ['base_joint', 'gimbal_joint', 'flywheel_joint']
		state.position = [msg.base_joint, msg.gimbal_joint, msg.rflywheel_joint] 
		self.joint_state_pub.publish(state)

if __name__ == '__main__':
	rospy.init_node('joint_state_publisher')
	try:
		jsp = JointStatePublisher()
	except:
		rospy.ROSInterruptException
	pass


