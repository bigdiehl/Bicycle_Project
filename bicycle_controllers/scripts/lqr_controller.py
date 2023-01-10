#!/usr/bin/env python3

import numpy as np
import control as c
import rospy
from controller import Controller, Integrator
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32

# Default values for v = 4
# A = np.array([[ 0.        ,  0.        ,  1.        ,  0.        ],
#               [ 0.        ,  0.        ,  0.        ,  1.        ],
#               [12.31395651, -0.87481776, -0.06793341,  0.65227584],
#               [ 0.1920345 ,  1.95833074,  1.06987266, -0.24431742]])

# B = np.array([[ 0.        ],
#               [ 0.        ],
#               [-0.15580437],
#               [ 2.45373858]])

# C = np.array([0, 1, 0, 0])

# D = np.array([[0.]])

# Q = np.array([[0, 0, 0, 0],
#               [0, 1, 0, 0],
#               [0, 0, 1, 0],
#               [0, 0, 0, 0]])

# R = 0.2

# Bug in rospy when specifying numpy array as default value.
# This wrapper function is a bandaid. Arrays from YAML are uploaded as strings
# and then converted into np.arrays dynamically
def get_param_array(param):
    try:
        val = rospy.get_param(param)
        # Convert string representation of array to np.array
        return eval(val)
    except KeyError:
        return None

class LQR_Controller (Controller) :
    def __init__(self):
        #super(LQR_Controller, self).__init__(A,B,C,D)
        self.type = 'integral' # integral, ideal, 

        self.A = get_param_array("whipple_controller/A")
        self.B = get_param_array("whipple_controller/B")
        self.C = get_param_array("whipple_controller/C")
        self.D = get_param_array("whipple_controller/D")

        if self.type == 'integral':
            self.Q = get_param_array("whipple_controller/lqr_integral/Q")
            self.R = get_param_array("whipple_controller/lqr_integral/R")
        elif self.type == 'ideal':
            self.Q = get_param_array("whipple_controller/lqr_ideal/Q")
            self.R = get_param_array("whipple_controller/lqr_ideal/R")
        else:
            raise ValueError("Bad self.type")

        # Matrices are read in as standard lists. Determine dimensions and convert to numpy arrays
        self.n_states = self.A.shape[0]
        self.m_inputs = self.B.shape[1]
        self.p_outputs = self.C.shape[0]

        self.wheel_radius = rospy.get_param("bicycle/wheel_radius", 0.35)

        self.delta_error_int = Integrator(0)
        self.control_rate = rospy.get_param("whipple_controller/update_rate", 100)
        self.Ts = 1.0/self.control_rate

        self.umax = rospy.get_param("whipple_controller/steering_torque_max", 20)

        self.compute_gains()

        self.int_pub = rospy.Publisher('bicycle/delta_int', Float32, queue_size=10)
        
        
    def compute_gains(self):
        if self.type == 'integral':
            # Assume control of the form u = -Kx - kI*integral(delta - delta_d)
            # NOTE - Make sure C is the correct shape. I.e. actually a matrix, not an array
            self.K, S, E = c.lqr(self.A, self.B, self.Q, self.R, integral_action=-self.C)
            
            # K is of dimensions (1, n_states + n_outputs). Split K into Kx and KI
            self.Kx = np.reshape(self.K[0, 0:4], (1,4))
            self.KI = self.K[0, 4]
            rospy.loginfo("Kx = " + self.Kx.__repr__())
            rospy.loginfo("kI = " + self.KI.__repr__())

        elif self.type == 'ideal':
            # Assume control of the form u = kr*delta - Kx 
            self.K, S, E = c.lqr(self.A, self.B, self.Q, self.R)
            self.kr = -1.0/(self.C.dot(np.linalg.inv(self.A-self.B.dot(self.K))).dot(self.B))
            rospy.loginfo("K = " + self.K.__repr__())
            rospy.loginfo("kr = " + self.kr.__repr__())


    def control(self, inputs, cmd_msg):
        if self.type == 'integral':
            return self.control_integral(inputs, cmd_msg)
        elif self.type == 'ideal':
            return self.control_ideal(inputs, cmd_msg)


    def control_integral(self, inputs, cmd_msg):
        """ PARAMS
            - inputs: dictionary of state/sensor information
            - cmd_msg: of type bicycle_msgs.BicycleCmd
        RETURN
            - Returns [torque_cmd, velocity_cmd, position_cmd] where each element
            is of type sensor_msgs.JointState or None. 
        """

        # Note: To align with Whipple model where for steer axis + is clockwise (as opposed 
        # to counterclockwise on our simulation model) we need to invert the following:
        #   * desired_steer_angle
        #   * delta
        #   * delta_dot
        #   * output torque, Td

        desired_steer_angle = cmd_msg.desired_steer_angle * -1
        #print("Received {:f} steer angle".format(cmd_msg.desired_steer_angle))

        state = np.array([inputs['phi'], -inputs['delta'], inputs['phidot'], -inputs['deltadot']])
        
        error = state[1] - desired_steer_angle

        Td = - self.Kx.dot(state) - self.KI*self.delta_error_int.integrate(error, self.Ts)

        Td = self.saturate(Td, self.umax, -self.umax)
        
        h = Header()
        h.stamp = rospy.Time.now()

        torque_cmd = JointState()
        torque_cmd.header = h
        torque_cmd.name = ["steering_joint"]

        torque_cmd.effort = [-Td]

        vel_cmd = JointState()
        vel_cmd.header = h
        vel_cmd.name = ["back_wheel_joint"]
        vel_cmd.velocity = [cmd_msg.desired_speed / self.wheel_radius] # covert from m/s to rad/s
        
        # For diagnostics, report integrator state
        self.int_pub.publish(self.delta_error_int.get_state())

        return [torque_cmd, vel_cmd, None]



    # NOTE - Assumes ideal model. No integrators
    def control_ideal(self, inputs, cmd_msg):
        """ PARAMS
            - inputs: dictionary of state/sensor information
            - cmd_msg: of type bicycle_msgs.BicycleCmd
        RETURN
            - Returns [torque_cmd, velocity_cmd, position_cmd] where each element
            is of type sensor_msgs.JointState or None. 
        """

        # Note: To align with Whipple model where for steer axis + is clockwise (as opposed 
        # to counterclockwise on our simulation model) we need to invert the following:
        #   * desired_steer_angle
        #   * delta
        #   * delta_dot
        #   * output torque, Td

        desired_steer_angle = cmd_msg.desired_steer_angle * -1
        #print("Received {:f} steer angle".format(cmd_msg.desired_steer_angle))

        state = np.array([inputs['phi'], -inputs['delta'], inputs['phidot'], -inputs['deltadot']])

        Td = self.kr*desired_steer_angle - self.K.dot(state)
        umax = 20
        Td = self.saturate(Td, self.umax, -self.umax)
        
        h = Header()
        h.stamp = rospy.Time.now()

        torque_cmd = JointState()
        torque_cmd.header = h
        torque_cmd.name = ["steering_joint"]

        torque_cmd.effort = [-Td]

        vel_cmd = JointState()
        vel_cmd.header = h
        vel_cmd.name = ["back_wheel_joint"]
        vel_cmd.velocity = [cmd_msg.desired_speed / self.wheel_radius] # covert from m/s to rad/s

        return [torque_cmd, vel_cmd, None]
        #return [None, vel_cmd, None]
    