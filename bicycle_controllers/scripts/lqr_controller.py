#!/usr/bin/env python3

import numpy as np
import control as c
import rospy
#from controller import Controller
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Default values for v = 4
A = np.array([[ 0.        ,  0.        ,  1.        ,  0.        ],
              [ 0.        ,  0.        ,  0.        ,  1.        ],
              [12.31395651, -0.87481776, -0.06793341,  0.65227584],
              [ 0.1920345 ,  1.95833074,  1.06987266, -0.24431742]])

B = np.array([[ 0.        ],
              [ 0.        ],
              [-0.15580437],
              [ 2.45373858]])

C = np.array([0, 1, 0, 0])

D = np.array([[0.]])

Q = np.array([[0, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 0]])

R = 0.2

# Bug in rospy when specifying numpy array as default value.
# This wrapper function is a bandaid. Arrays from YAML are uploaded as strings
# and then converted into np.arrays dynamically
def get_param_array(param, default):
    try:
        val = rospy.get_param(param)
        # Convert string representation of array to np.array
        return eval(val)
    except KeyError:
        return default

#class LQR_Controller (Controller) :
class LQR_Controller:
    def __init__(self):
        #super(LQR_Controller, self).__init__(A,B,C,D)

        self.A = get_param_array("controller/A", A)
        self.B = get_param_array("controller/B", B)
        self.C = get_param_array("controller/C", C)
        self.D = get_param_array("controller/D", D)
        self.Q = get_param_array("controller/Q", Q)
        self.R = get_param_array("controller/R", R)

        # Matrices are read in as standard lists. Determine dimensions and convert to numpy arrays
        self.n_states = self.A.shape[0]
        self.m_inputs = self.B.shape[1]
        self.p_outputs = self.C.shape[0]

        self.wheel_radius = rospy.get_param("bicycle/wheel_radius", 0.35)

        self.compute_gains()
        
    def compute_gains(self):
        """ Assume control of the form u = kr*delta - Kx """
        self.K, S, E = c.lqr(self.A, self.B, self.Q, self.R)
        self.kr = -1.0/(self.C.dot(np.linalg.inv(self.A-self.B.dot(self.K))).dot(self.B))
        rospy.logdebug("K = ", self.K)
        rospy.logdebug("kr = ", self.kr)

    def control(self, inputs, cmd_msg):
        """ PARAMS
            - inputs: dictionary of state/sensor information
            - cmd_msg: of type bicycle_msgs.BicycleCmd
        RETURN
            - Returns [torque_cmd, velocity_cmd, position_cmd] where each element
            is of type sensor_msgs.JointState or None. 
        """

        state = np.array([inputs['phi'], inputs['delta'], inputs['phidot'], inputs['deltadot']])

        Td = self.kr*cmd_msg.desired_steer_angle - self.K.dot(state)
        
        h = Header()
        h.stamp = rospy.Time.now()

        torque_cmd = JointState()
        torque_cmd.header = h
        torque_cmd.name = ["steering_joint"]
        torque_cmd.effort = [Td]

        vel_cmd = JointState()
        vel_cmd.header = h
        vel_cmd.name = ["back_wheel_joint"]
        vel_cmd.velocity = [cmd_msg.desired_speed / self.wheel_radius] # covert from m/s to rad/s

        return [None, vel_cmd, None]
    