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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from math import sin, cos

from inv_kin import *


class Joint_States():
    """
    Class to hold joint angle values.
    """
    def __init__(self, arm):
        self.valid = True
        self.arm = arm
        self.th1 = 0.0
        self.th2 = 0.0
        self.d3  = 0.0
        self.th4 = 0.0
        self.th5 = 0.0
        self.th6 = 0.0

    def print_vals(self):
        print "valid:\t" + str(self.valid) + \
              "\narm:\t" + str(self.arm) + \
              "\nth1:\t" + str(self.th1) + \
              "\nth2:\t" + str(self.th2) + \
              "\nd3:\t"  + str(self.d3)  + \
              "\nth4:\t" + str(self.th4) + \
              "\nth5:\t" + str(self.th5) + \
              "\nth6:\t" + str(self.th6) + "\n"


class Robot():
    """
    Class to hold raven state.
    """
    def __init__(self):
        self.num = 0
        self.T = np.matrix([[1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]])
        self.th_pos = [Joint_States('Gold'), Joint_States('Green')]  # kinematic theta values
        self.j_pos = [Joint_States('Gold'), Joint_States('Green')]  # actual joint positions
        self.ik_sol = np.matrix([[Joint_States('Gold') for i in range(8)],
                                [Joint_States('Green') for i in range(8)]])
        self.cube_size = 10
        self.i = -self.cube_size / 2
        self.j = -self.cube_size / 2
        self.k = -self.cube_size / 2

    def print_sols(self):
        for i in range(1):  # only testing left arm
            for j in range(8):
                if self.ik_sol[i, j].valid:
                    print "ik solution: " + str(j)
                    self.ik_sol[i, j].print_vals()

    def tf_update(self, tf_incr):
        self.T = self.T * tf_incr

    def inc_num(self):
        self.num = self.num + 1

# valid:	True
# arm:	Gold
# th1:	-0.613366933488
# th2:	1.29279947582
# d3:	-0.53978309596
# th4:	0.0
# th5:	-1.56678253559
# th6:	1.57542402281


pub_joint_abs = rospy.Publisher('/ravenII/set_joint_trajectory', JointTrajectory, queue_size=10)
traj_msg = JointTrajectory(joint_names=['shoulder_L', 'elbow_L', 'insertion_L', 'tool_roll_L',
                                'wrist_joint_L', 'grasper_joint_1_L', 'grasper_joint_2_L', ])
# given raven_automove msgs, do IK, update joint commands
def callback(data, Raven):
    # update desired Raven end effector position
    # Raven.T[0, 3] += -data.tf_incr[0].translation.z / 1000.0
    # Raven.T[1, 3] = -data.tf_incr[0].translation.y / 1000.0
    # Raven.T[2, 3] += data.tf_incr[0].translation.x / 1000.0
    # Raven.T[2, 3] = 0.03 + (data.tf_incr[0].translation.x / 500.0)

    # # raster scan
    # print "(" + str(Raven.i) + ", " + str(Raven.j) + ", " + str(Raven.k) + ")"
    # Raven.T[0, 3] = Raven.i / 100.0
    # Raven.T[1, 3] = Raven.j / 100.0
    # Raven.T[2, 3] = Raven.k / 100.0
    # Raven.k += 1
    # if Raven.i >= Raven.cube_size / 2:
    #     return
    # if Raven.k >= Raven.cube_size / 2:
    #     Raven.j += 1
    #     Raven.k = 0
    # if Raven.j >= Raven.cube_size / 2:
    #     Raven.i += 1
    #     Raven.j = 0

    arm = 'Gold'
    inv_kin_raven_git(Raven, arm)
    # Raven.print_sols()
    arm_idx = 0 if arm is 'Gold' else 1

    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.header.seq = Raven.num
    traj_msg.header.frame_id = 'world'

    # for i in range(8):
    #     Raven.ik_sol[arm_idx, i].print_vals()

    val = 0.20*sin(Raven.num/100.0)
    # print "val: " + str(val)
    # print "d3: " + str(Raven.j_pos[arm_idx].d3)
    # traj_pt = JointTrajectoryPoint(positions=[Raven.j_pos[arm_idx].th1, Raven.j_pos[arm_idx].th2,
    #                                           Raven.j_pos[arm_idx].d3,  Raven.j_pos[arm_idx].th4,
    #                                           Raven.j_pos[arm_idx].th5, Raven.j_pos[arm_idx].th6,
    #                                           Raven.j_pos[arm_idx].th6],
    #                                           time_from_start=rospy.Duration(0.0))

    # test IK solutions
    n = int(Raven.num / 600)
    if n > 7: return
    if not Raven.ik_sol[arm_idx, n].valid: Raven.num += 600
    print "n: " + str(n)
    traj_pt = JointTrajectoryPoint(positions=[Raven.ik_sol[arm_idx, n].th1, Raven.ik_sol[arm_idx, n].th2,
                                              Raven.ik_sol[arm_idx, n].d3,  Raven.ik_sol[arm_idx, n].th4,
                                              Raven.ik_sol[arm_idx, n].th5, Raven.ik_sol[arm_idx, n].th6,
                                              Raven.ik_sol[arm_idx, n].th6],
                                              time_from_start=rospy.Duration(0.0))

    # test limits
    # traj_pt = JointTrajectoryPoint(positions=[0.0, 0.0, val, 0.0, 0.0, 0.0, 0.0],
    #                                          time_from_start=rospy.Duration(0.0))

    # reset valid states
    for k in range(8):
        Raven.ik_sol[arm_idx, k].valid = True

    Raven.inc_num()
    traj_msg.points = [traj_pt]
    pub_joint_abs.publish(traj_msg)

def raven_automove_listener(Raven):
    """
    Receives raven_automove msgs, uses callback to update raven sim joint positions
    """
    # (-0.12, 0.04, 0.0) works with world rpy="0 ${PI/2} ${-PI/2}"/>, theta2joint
    # gazebo (x left, y out, z up)
    # raven (x out, y down, z right)
    # enter gazebo coords -> raven coords => (x, y, z) -> (y, -z, -x)
    x = -0.0
    y = 0.09
    z = -0.04
    Raven.T[0,3] = y  # -z  # x -> -z
    Raven.T[1,3] = -z  # -y
    Raven.T[2,3] = -x  # x  # z -> x
    # inv_kin_raven_git(Raven)
    rospy.init_node('msg_converter', anonymous=True)
    rospy.Subscriber("/automove_test", raven_automove, callback, Raven)
    rospy.spin()

if __name__ == '__main__':
    Raven = Robot()
    try:
        raven_automove_listener(Raven)
    except rospy.ROSInterruptException:
        pass
