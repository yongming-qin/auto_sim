#!/usr/bin/env python
""" test_inv_kin.py
Small script for testing the raven IK modules.
"""

__author__ = "Kyle Lindgren"
__version__ = "0.0.0"
__status__ = "Prototype"
__date__ = "Jan 19, 2018"

import rospy
import numpy as np
import tf
from math import sin, cos

from inv_kin import *
from msg_converter_abs import Robot, Joint_States

def test_IK(Raven):
    """
    Receives raven_automove msgs, uses callback to update raven sim joint positions
    """
    for i in range(-5, 5):
        print "\nx = " + str(-0.12 + i/100.0)
        Raven.T[0,3] = -0.12 + i/100.0
        Raven.T[1,3] = 0.04
        Raven.T[2,3] = -0.00
        inv_kin_raven_git(Raven, 'Gold')
    # Raven.print_sols()

if __name__ == '__main__':
    Raven = Robot()
    try:
        test_IK(Raven)
    except rospy.ROSInterruptException:
        pass
