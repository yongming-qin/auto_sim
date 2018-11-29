#!/usr/bin/env python
""" msg_converter.py
Subscribes to raven_automove, does IK, publishes joint commands to update gazebo sim.
"""

__author__ = "Kyle Lindgren"
__version__ = "0.0.0"
__status__ = "Prototype"
__date__ = "Jan 19, 2018"

import rospy
import numpy as np
from raven_imposter.msg import raven_automove
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Float64
import tf
from math import sin, cos

from inv_kin import *

class Robot():
    """
    Class to hold robot state.
    """
    def __init__(self):
        self.num = 0
        self.T = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

    def tf_update(self, tf_incr):
        self.T = self.T * tf_incr

    def inc_num(self):
        self.num = self.num + 1

pub_shoulder_L      = rospy.Publisher('/ravenII/shoulder_L_position_controller/command',
                                        Float64, queue_size=10)
pub_elbow_L         = rospy.Publisher('/ravenII/elbow_L_position_controller/command',
                                        Float64, queue_size=10)
pub_insertion_L     = rospy.Publisher('/ravenII/insertion_L_position_controller/command',
                                        Float64, queue_size=10)
pub_tool_roll_L     = rospy.Publisher('/ravenII/tool_roll_L_position_controller/command',
                                        Float64, queue_size=10)
pub_wrist_joint_L   = rospy.Publisher('/ravenII/wrist_joint_L_position_controller/command',
                                        Float64, queue_size=10)
pub_grasper_1_L     = rospy.Publisher('/ravenII/grasper_joint_1_L_position_controller/command',
                                        Float64, queue_size=10)
pub_grasper_2_L     = rospy.Publisher('/ravenII/grasper_joint_2_L_position_controller/command',
                                        Float64, queue_size=10)


# given raven_automove msgs, do IK, update joint commands
def callback(data, Raven):
    # update desired Raven end effector position
    Raven.T[0, 3] = Raven.T[0, 3] + data.tf_incr[0].translation.x
    Raven.T[1, 3] = Raven.T[1, 3] + data.tf_incr[0].translation.y
    Raven.T[2, 3] = Raven.T[2, 3] + data.tf_incr[0].translation.z

    sol_num = 0  # 0 or 1 for Raven_L_no_wrist
    # sol = inv_kin(Raven.T)
    sol = [[1, 1, 1, 1]]  # demo without performing IK
    sol[sol_num] = [Raven.T[0, 3]*x/1000 for x in sol[sol_num]]  # gazebo in meters, calcs mm
    # print sol

    Raven.inc_num()

    # publish separate joint commands to each *command topic
    pub_shoulder_L.publish(Float64(sol[sol_num][0]))
    pub_elbow_L.publish(Float64(sol[sol_num][1]))
    pub_insertion_L.publish(Float64(sol[sol_num][2]))
    pub_tool_roll_L.publish(Float64(sol[sol_num][3]))
    pub_wrist_joint_L.publish(Float64(sol[sol_num][3]))  # iterate for demo only, normally 0.0
    pub_grasper_1_L.publish(Float64(sol[sol_num][3]))  # iterate for demo only, normally 0.0
    pub_grasper_2_L.publish(Float64(-sol[sol_num][3]))  # iterate for demo only, normally 0.0

def raven_automove_listener(Raven):
    """
    Recieves raven_automove msgs, calls talker to update raven sim joint positions
    """
    th = np.pi / 2
    Raven.T = RotX4_N(-3*th) * RotY4_N(-th)
    Raven.T[0,3] = 0.2
    Raven.T[1,3] = 0.0
    Raven.T[2,3] = -2.2
    rospy.init_node('msg_converter', anonymous=True)
    rospy.Subscriber("/automove_test", raven_automove, callback, Raven)
    rospy.spin()

if __name__ == '__main__':
    Raven = Robot()
    try:
        raven_automove_listener(Raven)
    except rospy.ROSInterruptException:
        pass
