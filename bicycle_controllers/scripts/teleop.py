#!/usr/bin/env python3

""" Heavily based off of ROS's teleop_twist_keyboard.py code"""

import rospy
import roslib 
import sys
from select import select
import numpy as np

import termios 
import tty 

from bicycle_msgs.msg import BicycleCmd
from std_msgs.msg import Header

msg = """
Reading from the keyboard and publishing a BicycleCmd msg. 
----------------------------------------------------------
Controls are:
    w
a   s   d

where w/s increase/decrease desired_speed, and a/d increase/descrease 
desired_steer_angle. 

Commands are published on the bicycle/teleop_cmd topic at a rate of 10 Hz.

CTRL-C to quit
"""


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)    


class Bicycle_Teleop:
    def __init__(self, d0, v0):

        self.steer_angle = d0
        self.speed = v0

        # How much to update when a key press is recieved
        self.steer_update_rate = 1 # deg
        self.speed_update_rate = 0.1 # m/s
        self.valid_keys = ['a', 's', 'd', 'w']

    def update(self, key):

        cmd = BicycleCmd()
        h = Header()
        h.stamp = rospy.Time.now()
        cmd.header = h

        if key == 'a':
            self.steer_angle -= np.radians(self.steer_update_rate)
        elif key == 'd':
            self.steer_angle += np.radians(self.steer_update_rate)
        elif key == 'w':
            self.speed += self.speed_update_rate
        elif key == 's':
            self.speed -= self.speed_update_rate
        else:
            return None 

        cmd.desired_speed = self.speed
        cmd.desired_steer_angle = self.steer_angle

        return cmd

    def print_cmd(self, cmd):
        print("Current command: (v,d) = ({:.2f}, {:.2f})".format(self.speed, np.degrees(self.steer_angle)))

if __name__=="__main__":
    rospy.loginfo(msg)

    settings = saveTerminalSettings()

    rospy.init_node('teleop_bicycle')
    cmd_pub = rospy.Publisher("bicycle/command", BicycleCmd, queue_size=10)

    key_timeout = rospy.get_param("~key_timeout", 0.5)

    d0 = rospy.get_param("bicycle/initial_conditions/delta", 0)
    v0 = rospy.get_param("bicycle/initial_conditions/vel", 0)

    # At startup, command to maintain initial conditions
    last_cmd = BicycleCmd()
    h = Header()
    h.stamp = rospy.Time.now()
    last_cmd.header = h
    last_cmd.desired_speed = v0
    last_cmd.desired_steer_angle = d0

    teleop = Bicycle_Teleop(d0, v0)

    while not rospy.is_shutdown():
        key = getKey(settings, key_timeout)
        
        if key in teleop.valid_keys:
            #print("Key is: ", key)
            cmd = teleop.update(key)
            teleop.print_cmd(cmd)

            if cmd is not None:
                cmd_pub.publish(cmd)
                last_cmd = cmd

        elif (key == '\x03'):
            # Exit if Ctrl-C received
            break

        # Publish the last cmd while idle. Will effectively publish at 1.0/key_timeout Hz
        elif last_cmd is not None:
            h = Header()
            h.stamp = rospy.Time.now()
            last_cmd.header = h
            cmd_pub.publish(last_cmd)

        
        

 