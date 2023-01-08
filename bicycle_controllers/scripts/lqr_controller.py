import numpy as np
import control as c
import rospy
from bicycle_controller import Controller
from sensor_msgs.msg import JointState

class LQR_Controller (Controller) :
    def __init__(self, A, B, C, D, Q, R):
        #super(LQR_Controller, self).__init__(A,B,C,D)

        # Read in linear model state space and cost matrices
        self.A = rospy.get_param("controller/A")
        self.B = rospy.get_param("controller/B")
        self.C = rospy.get_param("controller/C")
        self.D = rospy.get_param("controller/D")

        self.Q = rospy.get_param("controller/Q")
        self.R = rospy.get_param("controller/R")

        # Matrices are read in as standard lists. Determine dimensions and convert to numpy arrays
        self.n_states = np.sqrt(len(self.A))
        self.m_inputs = len(self.B) / self.n_states
        self.p_outputs = len(self.C) / self.n_states

        self.A = np.resize(np.array(self.A), (self.n_states, self.n_states))
        self.B = np.resize(np.array(self.B), (self.n_states, self.m_inputs))
        self.C = np.resize(np.array(self.C), (self.p_outputs, self.n_states))
        self.D = np.resize(np.array(self.D), (self.p_outputs, self.m_inputs))

        self.Q = np.resize(np.array(self.D), (self.p_outputs, self.m_inputs))
        self.R = np.resize(np.array(self.D), (self.p_outputs, self.m_inputs))

        self.Q = np.resize(np.array(self.Q), (self.n_states, self.n_states)) 
        self.R = np.array(self.R)

        self.compute_gains()
        
    def compute_gains(self, K, R):
        """ Assume control of the form u = kr - Kx """
        self.K, S, E = c.lqr(self.A, self.B, self.Q, self.R)
        self.kr = -1.0/(self.C.dot(np.linalg.inv(self.A-self.B.dot(K))).dot(self.B))

    def control(self, inputs, cmd_msg):
        """ PARAMS
            - inputs: dictionary of state/sensor information
            - cmd_msg: of type bicycle_msgs.BicycleCmd
        RETURN
            - Returns [torque_cmd, velocity_cmd, position_cmd] where each element
            is of type sensor_msgs.JointState or None. 
        """

        state = np.array([inputs['phi'], inputs['delta'], inputs['phidot'], inputs['deltadot']])


        Td = self.kr - self.K.dot(state)

        torque_cmd = JointState()
        torque_cmd.name = ["steering_joint"]
        torque_cmd.effort = Td

        vel_cmd = JointState()
        vel_cmd.name = ["back_wheel_joint"]
        vel_cmd.velocity = cmd_msg.desired_speed

        return [torque_cmd, vel_cmd, None]
    