#!/usr/bin/env python3

""" Flywheel balance controller """

import numpy as np
import control as c
import rospy
from controller import Controller
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class Flywheel_Controller (Controller) :

    def __init__(self):
        pass


    def control(self, inputs, cmd_msg):
        
        pass