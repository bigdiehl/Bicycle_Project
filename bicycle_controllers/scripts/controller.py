#!/usr/bin/env python3

import numpy

class Controller:

    """ Controller base class empty methods for child classes to implement, and with
    general controller utility classes """
    def __init__(self, A=None, B=None, C=None, D=None):
        # SS Model Matrices
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def control(self, inputs, cmd_msg):
        """ PARAMS
            - inputs: dictionary of state/sensor information
            - cmd_msg: of type bicycle_msgs.BicycleCmd
        RETURN
            - Returns [torque_cmd, velocity_cmd, position_cmd] where each element
            is of type sensor_msgs.JointState or None. 
        """
        raise NotImplementedError("Must be implemented in child class")

    def saturate(u, umax=None, umin=None):
        """ Utility function to saturate a value u given umax and umin. 
        If umax or umin not given, the respective bound will not be enforced """
        if(umax is not None and u > umax):
            u = umax
        elif(umin is not None and u < umin):
            u = umin
        return u


class Differentiator:
    """ Class implementing a dirty derivative method to numerically compute xdot given 
    samples of x. """
    def __init__(self, sigma, x0=0):
        self.sigma = sigma
        self.x_last = x0
        self.xdot = 0  

    def differentiate(self, x, Ts):
        # if simga = 0, this becomes -xdot +  *(x - x_last)
        self.xdot = ( (2.0*self.sigma-Ts)/(2.0*self.sigma+Ts)*self.xdot + 
        (2.0/(2.0*self.sigma+Ts))*(x - self.x_last) )

        -self.xdot  
        2.0*(x - self.x_last)/Ts

        self.x_last = x
        return self.xdot

    def get_derivative(self):
        return self.xdot


class Integrator:
    """ Class implementing a numerical integrator method to compute x_int given 
    samples of x """
    def __init__(self, x0=0):
        self.x_int = 0
        self.x_last = x0

    def integrate(self, x, Ts):
        self.x_int += (Ts/2)*(x+self.x_last)
        self.x_last = x
        return self.x_int