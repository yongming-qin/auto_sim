#!/usr/bin/env python
""" inv_kin.py
Does IK for raven, returning joint positions.
"""

__author__ = "Kyle Lindgren"
__version__ = "0.0.0"
__status__ = "Prototype"
__date__ = "Jan 19, 2018"


import rospy
import numpy as np
from sklearn.preprocessing import normalize
from math import sqrt, atan2, cos, sin, acos, asin

#  Declare the parameters
a_1 = 1.309  # github.com/uw-biorobotics/raven2/blob/master/include/raven/r2_kinematics.h
a_2 = 0.9076
# d_4 = -0.47  # raven diamond tool
d_4 = -0.45869  # raven square tool
l_w = 0.013
GM1 = sin(a_1)
GM2 = cos(a_1)
GM3 = sin(a_2)
GM4 = cos(a_2)
pi = np.pi
eps = 1.0e-05
d2r = pi / 180.0

# //------------------------------------------------------------------------------------------
# // Conversion of J to Theta /// Theta 2 J
# // J represents the physical robot joint angles.
# // Theta is used by the kinematics.
# // Theta convention was easier to solve the equations, while J was already coded in software.
# //-----------------------------------------------------------------------------------------

TH1_J0_L  = 205    # add this to J0 to get \theta1 (in deg)
TH2_J1_L  = 180    # add this to J1 to get \theta2 (in deg)
D3_J2_L   = -d_4   # add this to J2 to get d3 (in meters????)
TH4_J3_L  = 0     # add this to J3 to get \theta4 (in deg)
TH5_J4_L  = -90    # add this to J4 to get \theta5 (in deg)
TH6A_J5_L = 0     # add this to J5 to get \theta6a (in deg)
TH6B_J6_L = 0     # add this to J6 to get \theta6b (in deg)

# DH parameters
class DH():
    """
    Holds dh parameters -> making a class allows multiple instances.
    """
    def __init__(self):
        self.dh_alpha = np.matrix([[0.0, a_1, pi - a_2, 0.0, pi/2.0, pi/2.0],
                                   [pi,  a_1,   a_2,    0.0, pi/2.0, pi/2.0]])
        self.dh_a = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, l_w],
                               [0.0, 0.0, 0.0, 0.0, 0.0, l_w]])
        self.dh_d = np.matrix([[0.0, 0.0, 0.0, d_4, 0.0, 0.0],
                               [0.0, 0.0, 0.0, d_4, 0.0, 0.0]])
        self.dh_theta = np.matrix([[0.0, 0.0,  pi/2.0, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, -pi/2.0, 0.0, 0.0, 0.0]])


def getFKTransform(DH, a, b, arm_idx):
    if b <= a or b is 0:
        print "Invalid start/end indices for forward kinematics calculation"

    xx =  cos(DH.dh_theta[arm_idx, a])
    xy = -sin(DH.dh_theta[arm_idx, a])
    xz = 0.0
    yx = sin(DH.dh_theta[arm_idx, a])*cos(DH.dh_alpha[arm_idx, a])
    yy = cos(DH.dh_theta[arm_idx, a])*cos(DH.dh_alpha[arm_idx, a])
    yz = -sin(DH.dh_alpha[arm_idx, a])
    zx = sin(DH.dh_theta[arm_idx, a])*sin(DH.dh_alpha[arm_idx, a])
    zy = cos(DH.dh_theta[arm_idx, a])*sin(DH.dh_alpha[arm_idx, a])
    zz = cos(DH.dh_alpha[arm_idx, a])

    px =  DH.dh_a[arm_idx, a]
    py = -sin(DH.dh_alpha[arm_idx, a])*DH.dh_d[arm_idx, a]
    pz =  cos(DH.dh_alpha[arm_idx, a])*DH.dh_d[arm_idx, a]

    xf = np.matrix([[xx, xy, xz, px],
                    [yx, yy, yz, py],
                    [zx, zy, zz, pz],
                    [0.0, 0.0, 0.0, 1.0]])

    if b > a+1:
        xf = np.matmul(xf, getFKTransform(DH, a+1, b, arm_idx))

    return xf

def getFKTransform_mat(DH, a, arm_idx):
    xx =  cos(DH.dh_theta[arm_idx, a])
    xy = -sin(DH.dh_theta[arm_idx, a])
    xz = 0.0
    yx = sin(DH.dh_theta[arm_idx, a])*cos(DH.dh_alpha[arm_idx, a])
    yy = cos(DH.dh_theta[arm_idx, a])*cos(DH.dh_alpha[arm_idx, a])
    yz = -sin(DH.dh_alpha[arm_idx, a])
    zx = sin(DH.dh_theta[arm_idx, a])*sin(DH.dh_alpha[arm_idx, a])
    zy = cos(DH.dh_theta[arm_idx, a])*sin(DH.dh_alpha[arm_idx, a])
    zz = cos(DH.dh_alpha[arm_idx, a])

    px =  DH.dh_a[arm_idx, a]
    py = -sin(DH.dh_alpha[arm_idx, a])*DH.dh_d[arm_idx, a]
    pz =  cos(DH.dh_alpha[arm_idx, a])*DH.dh_d[arm_idx, a]

    xf = np.matrix([[xx, xy, xz, px],
                    [yx, yy, yz, py],
                    [zx, zy, zz, pz],
                    [0.0, 0.0, 0.0, 1.0]])
    return xf


def inv_kin_raven_git(Raven, arm):
    """
    Perform IK for raven, returning joint states.
    """
    if(Raven.T.shape != (4,4)):
        print "Input must be 4x4 numpy matrix"
        quit()

    arm_idx = 0 if arm is 'Gold' else 1

    # val = 0.2*sin(Raven.num/100.0)
    # dh_d[arm_idx, 2] = val
    # print "FK transform 0->6:\n" + str(getFKTransform(0, 6, arm_idx))
    # transform = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(getFKTransform_mat(0, arm_idx),
        # getFKTransform_mat(1, arm_idx)), getFKTransform_mat(2, arm_idx)),
        # getFKTransform_mat(3, arm_idx)), getFKTransform_mat(4, arm_idx)),
        # getFKTransform_mat(5, arm_idx))
    # print "FK transform 0->5:\n" + str(transform)
    # return

    Raven_DH = DH()

    # step 1, compute P5
    T60 = np.linalg.inv(Raven.T)

    p6rcm = np.transpose([T60[0, 3], T60[1, 3], 0.0])  # projection onto xy plane

    p05 = [[0.0 for i in range(3)] for j in range(8)]

    for i in range(2):
        if np.linalg.norm(p6rcm) > 0.0:
            p65 = np.transpose((-1+2*i) * l_w * np.asarray(p6rcm / np.linalg.norm(p6rcm)))
        else:
            p65 = np.asarray([0.0, 0.0, 0.0])
        p05[4*i] = p05[4*i+1] = p05[4*i+2] = p05[4*i+3] = np.matmul(Raven.T[:3, :3], p65) +\
                                                                np.transpose(Raven.T[:3, 3])

    # step 2, compute displacement of prismatic joint d3
    for i in range(2):
        # print "p05: " + str(p05[4*i])
        insertion = np.linalg.norm(p05[4*i])
        # print "insertion value: " + str(insertion)

        if insertion < l_w:
            print "WARNING: mechanism at RCM singularity(Lw: " + str(l_w) + ", ins: " +\
                    str(insertion) + "). IK failing."

        Raven.ik_sol[arm_idx, 4*i + 0].d3 = -d_4 - insertion
        Raven.ik_sol[arm_idx, 4*i + 1].d3 = -d_4 - insertion
        Raven.ik_sol[arm_idx, 4*i + 2].d3 = -d_4 + insertion
        Raven.ik_sol[arm_idx, 4*i + 3].d3 = -d_4 + insertion

    # step 3, calc theta 2
    for i in range(0, 8, 2):
        # print p05[i]
        z0p5 = p05[i][0, 2]

        d = Raven.ik_sol[arm_idx, i].d3 + d_4
        # print "z0p5: " + str(z0p5) + "\td3: " + str(d)
        cth2 = 0.0

        if not arm_idx: cth2 = 1 / (GM1*GM3) * ((-z0p5 / d) - GM2*GM4)
        else: cth2 = 1 / (GM1*GM3) * ((z0p5 / d) + GM2*GM4)

        # print "cth2: " + str(cth2)
        if cth2 > 1 and cth2 < 1+eps: cth2 =  1
        elif cth2 < -1 and cth2 > -1-eps: cth2 = -1

        if cth2 > 1 or cth2 < -1:
            Raven.ik_sol[arm_idx, i].valid = False
            Raven.ik_sol[arm_idx, i+1].valid = False
            print "cannot find th2"
        else:
            Raven.ik_sol[arm_idx, i].th2 = acos(cth2)
            Raven.ik_sol[arm_idx, i+1].th2 = -acos(cth2)

    # Raven.print_sols()

    # step 4, compute theta 1
    for i in range(8):
        if not Raven.ik_sol[arm_idx, i].valid:
            continue

        cth2 = cos(Raven.ik_sol[arm_idx, i].th2)
        sth2 = sin(Raven.ik_sol[arm_idx, i].th2)
        d = Raven.ik_sol[arm_idx, i].d3 + d_4
        BB1 = sth2*GM3
        BB2 = 0.0
        Bmx = np.empty([3, 3])
        xyp05 = p05[i][:]
        xyp05[0, 2] = 0.0
        # print xyp05
        if not arm_idx:
            BB2 = cth2*GM2*GM3 - GM1*GM4
            Bmx = np.matrix([[BB1, BB2, 0.0],
                             [-BB2, BB1, 0.0],
                             [0.0, 0.0, 1.0]])
        else:
            BB2 = cth2*GM2*GM3 + GM1*GM4
            Bmx = np.matrix([[BB1, BB2, 0.0],
                             [BB2, -BB1, 0.0],
                             [0.0, 0.0, 1.0]])

        scth1 = np.matmul(np.linalg.inv(Bmx), np.transpose(np.multiply(xyp05, 1.0/d)))
        # print scth1
        Raven.ik_sol[arm_idx, i].th1 = atan2(scth1[1], scth1[0])
        # print Raven.ik_sol[arm_idx, i].th1
    Raven_DH.dh_theta[arm_idx, 0] = Raven.ik_sol[arm_idx, i].th1
    Raven_DH.dh_theta[arm_idx, 1] = Raven.ik_sol[arm_idx, i].th2
    Raven_DH.dh_d[arm_idx, 2] = Raven.ik_sol[arm_idx, i].d3

    # print "FK transform 0->3:\n" + str(getFKTransform(Raven_DH, 0, 3, arm_idx))
    # print "FK transform 0->4:\n" + str(getFKTransform(Raven_DH, 0, 4, arm_idx))
    # print "FK transform 0->5:\n" + str(getFKTransform(Raven_DH, 0, 5, arm_idx))
    # print "FK transform 0->6:\n" + str(getFKTransform(Raven_DH, 0, 6, arm_idx))
    # Raven.print_sols()

    # step 5, get theta 4, 5, 6
    for i in range(8):
        if not Raven.ik_sol[arm_idx, i].valid:
            continue

        Raven_DH.dh_theta[arm_idx, 0] = Raven.ik_sol[arm_idx, i].th1
        Raven_DH.dh_theta[arm_idx, 1] = Raven.ik_sol[arm_idx, i].th2
        Raven_DH.dh_d[arm_idx, 2] = Raven.ik_sol[arm_idx, i].d3
        T03 = getFKTransform(Raven_DH, 0, 3, arm_idx)
        # print T03
        T36 = np.matmul(np.linalg.inv(T03), Raven.T)
        # print T36

        c5 = -T36[2, 2]
        s5 = (T36[3, 2] - d_4) / l_w

        # compute theta 4
        if abs(c5) > eps:
            c4 = T36[3, 0] / (l_w * c5)
            s4 = T36[3, 1] / (l_w * c5)
        else:
            c4 = T36[0, 2] / s5
            s4 = T36[1, 2] / s5

        Raven.ik_sol[arm_idx, i].th4 = atan2(s4, c4)
        # print Raven.ik_sol[arm_idx, i].th4

        # compute theta 5
        Raven.ik_sol[arm_idx, i].th5 = atan2(s5, c5)

        # compute theta 6
        if abs(s5) > eps:
            c6 = T36[2, 0] / s5
            s6 = -T36[2, 1] / s5
        else:
            Raven_DH.dh_theta[arm_idx, 3] = Raven.ik_sol[arm_idx, i].th4
            Raven_DH.dh_theta[arm_idx, 4] = Raven.ik_sol[arm_idx, i].th5
            T05 = np.matmul(T03, getFKTransform(Raven_DH, 3, 5, arm_idx))
            T56 = np.matmul(np.linalg.inv(T05), Raven.T)
            c6 = T56[0, 0]
            s6 = T56[2, 0]
        Raven.ik_sol[arm_idx, i].th6 = atan2(s6, c6)


    # assign raven joint values to ik solution closest to current config
    min_idx = check_solutions(Raven, arm_idx)
    # print min_idx
    if min_idx > -1:
        # Raven.th_pos[arm_idx] = Raven.ik_sol[arm_idx, min_idx]  # causes major breakage
        Raven.th_pos[arm_idx].th1 = Raven.ik_sol[arm_idx, min_idx].th1
        Raven.th_pos[arm_idx].th2 = Raven.ik_sol[arm_idx, min_idx].th2
        Raven.th_pos[arm_idx].d3 = Raven.ik_sol[arm_idx, min_idx].d3
        Raven.th_pos[arm_idx].th4 = Raven.ik_sol[arm_idx, min_idx].th4
        Raven.th_pos[arm_idx].th5 = Raven.ik_sol[arm_idx, min_idx].th5
        Raven.th_pos[arm_idx].th6 = Raven.ik_sol[arm_idx, min_idx].th6
        theta2joint(Raven, arm_idx)

        # Raven.j_pos[arm_idx].th1 = Raven.ik_sol[arm_idx, min_idx].th1
        # Raven.j_pos[arm_idx].th2 = Raven.ik_sol[arm_idx, min_idx].th2
        # Raven.j_pos[arm_idx].d3  = Raven.ik_sol[arm_idx, min_idx].d3
        # Raven.j_pos[arm_idx].th4 = Raven.ik_sol[arm_idx, min_idx].th4
        # Raven.j_pos[arm_idx].th5 = Raven.ik_sol[arm_idx, min_idx].th5
        # Raven.j_pos[arm_idx].th6 = Raven.ik_sol[arm_idx, min_idx].th6

        # Raven_DH.dh_theta[arm_idx, 0] = Raven.ik_sol[arm_idx, min_idx].th1
        # Raven_DH.dh_theta[arm_idx, 1] = Raven.ik_sol[arm_idx, min_idx].th2
        # Raven_DH.dh_d[arm_idx, 2] = Raven.ik_sol[arm_idx, min_idx].d3
        # Raven_DH.dh_theta[arm_idx, 3] = Raven.ik_sol[arm_idx, min_idx].th4
        # Raven_DH.dh_theta[arm_idx, 4] = Raven.ik_sol[arm_idx, min_idx].th5
        # Raven_DH.dh_theta[arm_idx, 5] = Raven.ik_sol[arm_idx, min_idx].th6
        # print "FK transform 0->5:\n" + str(getFKTransform(Raven_DH, 0, 5, arm_idx))
    else:
        print "No valid solution found."
    # Raven.th_pos[arm_idx].print_vals()

    # print "FK transform 0->6:\n" + str(getFKTransform(0, 6, arm_idx))


# check the inverse kinematic solutions
def check_solutions(Raven, arm_idx):
    min_err = 99999999.0  # 32765
    idx = -1
    ret_roll = False

    for i in range(8):
        # Raven.ik_sol[arm_idx, i].print_vals()
        if not Raven.ik_sol[arm_idx, i].valid:
            continue

        # if Raven.ik_sol[arm_idx, i].th1 > 0.2 or Raven.ik_sol[arm_idx, i].th1 < -2.0:
        #     Raven.ik_sol[arm_idx, i].valid = False
        # if Raven.ik_sol[arm_idx, i].th2 > 2.4 or Raven.ik_sol[arm_idx, i].th2 < -0.7:
        #     Raven.ik_sol[arm_idx, i].valid = False
        # if Raven.ik_sol[arm_idx, i].d3 > 0.1 or Raven.ik_sol[arm_idx, i].d3 < -0.17:
        #     Raven.ik_sol[arm_idx, i].valid = False

        if Raven.ik_sol[arm_idx, i].th1 > pi:
            Raven.ik_sol[arm_idx, i].th1 -= 2*pi
        if Raven.ik_sol[arm_idx, i].th1 < -pi:
            Raven.ik_sol[arm_idx, i].th1 += 2*pi
        if Raven.ik_sol[arm_idx, i].th2 > pi:
            Raven.ik_sol[arm_idx, i].th2 -= 2*pi
        if Raven.ik_sol[arm_idx, i].th2 < -pi:
            Raven.ik_sol[arm_idx, i].th2 += 2*pi

        # -2.7 < j_pos1 < 0.0 => 0.88 < th_pos1 < 3.58
        # -2.3 < j_pos2 < 0.0 =>
        if Raven.ik_sol[arm_idx, i].th1 > 3.58 or Raven.ik_sol[arm_idx, i].th1 < 0.88:
            Raven.ik_sol[arm_idx, i].valid = False
        if Raven.ik_sol[arm_idx, i].th2 > 3.14 or Raven.ik_sol[arm_idx, i].th2 < 0.84:
            Raven.ik_sol[arm_idx, i].valid = False
        if Raven.ik_sol[arm_idx, i].d3 > 0.48 or Raven.ik_sol[arm_idx, i].d3 < 0.3:
            Raven.ik_sol[arm_idx, i].valid = False
        # print "updated validity: " + str(Raven.ik_sol[arm_idx, i].valid)

        # if abs(Raven.ik_sol[arm_idx, i].th4) > 3.14: Raven.ik_sol[arm_idx, i].valid = False
        # if abs(Raven.ik_sol[arm_idx, i].th5) > 3.14: Raven.ik_sol[arm_idx, i].valid = False
        # if abs(Raven.ik_sol[arm_idx, i].th6) > 3.14: Raven.ik_sol[arm_idx, i].valid = False

    # joint2theta(Raven, arm_idx)

    for j in range(8):
        s2_err = 0.0
        # print "s2_err: " + str(s2_err) + "\tmin_err: " + str(min_err) + "\tmin_idx: " + str(idx)
        if not Raven.ik_sol[arm_idx, j].valid:
            continue
        # rollover = False
        # # not sure why a rollover flag is used -- should be one for each solution
        # if abs(Raven.th_pos[arm_idx].th4) - Raven.ik_sol[arm_idx, j].th4 > 300.0 * d2r:
        #     rollover = True
        # if Raven.th_pos[arm_idx].th4 > Raven.ik_sol[arm_idx, j].th4:
        #     Raven.ik_sol[arm_idx, j].th4 += 2*pi
        # else:
        #     Raven.ik_sol[arm_idx, j].th4 -= 2*pi
        s2_err = s2_err + (Raven.th_pos[arm_idx].th1 - Raven.ik_sol[arm_idx, j].th1)**2
        s2_err = s2_err + (Raven.th_pos[arm_idx].th2 - Raven.ik_sol[arm_idx, j].th2)**2
        s2_err = s2_err + (100*(Raven.th_pos[arm_idx].d3 - Raven.ik_sol[arm_idx, j].d3))**2
        s2_err = s2_err + (Raven.th_pos[arm_idx].th4 - Raven.ik_sol[arm_idx, j].th4)**2
        s2_err = s2_err + (Raven.th_pos[arm_idx].th5 - Raven.ik_sol[arm_idx, j].th5)**2
        s2_err = s2_err + (Raven.th_pos[arm_idx].th6 - Raven.ik_sol[arm_idx, j].th6)**2
        # print "s2_err: " + str(s2_err) + "\tmin_err: " + str(min_err) + "\tmin_idx: " + str(idx)
        # print "j = " + str(j)
        if s2_err < min_err:
            min_err = s2_err
            idx = j
            # ret_roll = rollover

    return idx


def joint2theta(Raven, arm_idx):
    """
    Converts the inverse kinematic solution to the thetas
    """
    if not arm_idx:
        Raven.th_pos[arm_idx].th1 = Raven.j_pos[arm_idx].th1 + TH1_J0_L * d2r
        Raven.th_pos[arm_idx].th2 = Raven.j_pos[arm_idx].th2 + TH2_J1_L * d2r
        Raven.th_pos[arm_idx].d3  = Raven.j_pos[arm_idx].d3  + D3_J2_L
        Raven.th_pos[arm_idx].th4 = Raven.j_pos[arm_idx].th4 + TH4_J3_L * d2r
        Raven.th_pos[arm_idx].th5 = Raven.j_pos[arm_idx].th5 + TH5_J4_L * d2r
        Raven.th_pos[arm_idx].th6 = Raven.j_pos[arm_idx].th6 + TH6A_J5_L * d2r

        if Raven.th_pos[arm_idx].th1 > pi:
            Raven.th_pos[arm_idx].th1 -= 2*pi
        if Raven.th_pos[arm_idx].th1 < -pi:
            Raven.th_pos[arm_idx].th1 += 2*pi
        if Raven.th_pos[arm_idx].th2 > pi:
            Raven.th_pos[arm_idx].th2 -= 2*pi
        if Raven.th_pos[arm_idx].th2 < -pi:
            Raven.th_pos[arm_idx].th2 += 2*pi
        if Raven.th_pos[arm_idx].th4 > pi:
            Raven.th_pos[arm_idx].th4 -= 2*pi
        if Raven.th_pos[arm_idx].th4 < -pi:
            Raven.th_pos[arm_idx].th4 += 2*pi
        if Raven.th_pos[arm_idx].th5 > pi:
            Raven.th_pos[arm_idx].th5 -= 2*pi
        if Raven.th_pos[arm_idx].th5 < -pi:
            Raven.th_pos[arm_idx].th5 += 2*pi
        if Raven.th_pos[arm_idx].th6 > pi:
            Raven.th_pos[arm_idx].th6 -= 2*pi
        if Raven.th_pos[arm_idx].th6 < -pi:
            Raven.th_pos[arm_idx].th6 += 2*pi

def theta2joint(Raven, arm_idx):
    """
    Converts theta values to joint angles.
    """
    if not arm_idx:
        # print "theta2joint entered"
        Raven.j_pos[arm_idx].th1 = Raven.th_pos[arm_idx].th1 - (TH1_J0_L * d2r)
        Raven.j_pos[arm_idx].th2 = Raven.th_pos[arm_idx].th2 - (TH2_J1_L * d2r)
        Raven.j_pos[arm_idx].d3  = Raven.th_pos[arm_idx].d3  - D3_J2_L
        Raven.j_pos[arm_idx].th4 = Raven.th_pos[arm_idx].th4 - (TH4_J3_L * d2r)
        Raven.j_pos[arm_idx].th5 = Raven.th_pos[arm_idx].th5 - (TH5_J4_L * d2r)
        Raven.j_pos[arm_idx].th6 = Raven.th_pos[arm_idx].th6 - (TH6A_J5_L * d2r)

        if Raven.j_pos[arm_idx].th1 > pi:
            Raven.j_pos[arm_idx].th1 -= 2*pi
        if Raven.j_pos[arm_idx].th1 < -pi:
            Raven.j_pos[arm_idx].th1 += 2*pi
        if Raven.j_pos[arm_idx].th2 > pi:
            Raven.j_pos[arm_idx].th2 -= 2*pi
        if Raven.j_pos[arm_idx].th2 < -pi:
            Raven.j_pos[arm_idx].th2 += 2*pi
        if Raven.j_pos[arm_idx].th4 > pi:
            Raven.j_pos[arm_idx].th4 -= 2*pi
        if Raven.j_pos[arm_idx].th4 < -pi:
            Raven.j_pos[arm_idx].th4 += 2*pi
        if Raven.j_pos[arm_idx].th5 > pi:
            Raven.j_pos[arm_idx].th5 -= 2*pi
        if Raven.j_pos[arm_idx].th5 < -pi:
            Raven.j_pos[arm_idx].th5 += 2*pi
        if Raven.j_pos[arm_idx].th6 > pi:
            Raven.j_pos[arm_idx].th6 -= 2*pi
        if Raven.j_pos[arm_idx].th6 < -pi:
            Raven.j_pos[arm_idx].th6 += 2*pi

def inv_kin_IKBT(T):
    """
    Perform IK for raven, returning joint states.
    """
    if(T.shape != (4,4)):
        print "Input must be 4x4 numpy matrix"
        quit()

    #define the input vars
    r_11 = T[0,0]
    r_12 = T[0,1]
    r_13 = T[0,2]
    r_21 = T[1,0]
    r_22 = T[1,1]
    r_23 = T[1,2]
    r_31 = T[2,0]
    r_32 = T[2,1]
    r_33 = T[2,2]
    Px   = T[0,3]
    Py   = T[1,3]
    Pz   = T[2,3]

    # print T, Px, Py, Pz
    #Variable:  th_1
    # print r_13**2*sin(a_1)**2
    # print r_23**2*sin(a_1)**2
    # print (r_33*cos(a_1) + cos(a_2))**2
    th_1s1 = atan2(-r_13*sin(a_1), r_23*sin(a_1)) + atan2(sqrt(r_13**2*sin(a_1)**2 +
                r_23**2*sin(a_1)**2 - (r_33*cos(a_1) + cos(a_2))**2), r_33*cos(a_1) + cos(a_2))
    th_1s2 = atan2(-r_13*sin(a_1), r_23*sin(a_1)) + atan2(-sqrt(r_13**2*sin(a_1)**2 +
                r_23**2*sin(a_1)**2 - (r_33*cos(a_1) + cos(a_2))**2), r_33*cos(a_1) + cos(a_2))

    #Variable:  th_2
    th_2s2 = atan2((r_13*cos(th_1s2) + r_23*sin(th_1s2))/sin(a_2),
                        -(r_33 + cos(a_1)*cos(a_2))/(sin(a_1)*sin(a_2)))
    th_2s1 = atan2((r_13*cos(th_1s1) + r_23*sin(th_1s1))/sin(a_2),
                        -(r_33 + cos(a_1)*cos(a_2))/(sin(a_1)*sin(a_2)))

    #Variable:  d_3
    d_3s1 = (Px*cos(th_1s2) + Py*sin(th_1s2) - d_4*sin(a_2)*sin(th_2s2))/(sin(a_2)*sin(th_2s2))
    d_3s2 = (Px*cos(th_1s1) + Py*sin(th_1s1) - d_4*sin(a_2)*sin(th_2s1))/(sin(a_2)*sin(th_2s1))

    #Variable:  th_4
    th_4s1 = atan2(-(r_12*sin(a_1)*sin(th_1s1) - r_22*sin(a_1)*cos(th_1s1) +
                        r_32*cos(a_1))/sin(a_2), (r_11*sin(a_1)*sin(th_1s1) -
                        r_21*sin(a_1)*cos(th_1s1) + r_31*cos(a_1))/sin(a_2))
    th_4s2 = atan2(-(r_12*sin(a_1)*sin(th_1s2) - r_22*sin(a_1)*cos(th_1s2) +
                        r_32*cos(a_1))/sin(a_2), (r_11*sin(a_1)*sin(th_1s2) -
                        r_21*sin(a_1)*cos(th_1s2) + r_31*cos(a_1))/sin(a_2))

    # package the solutions into a list for each set
    solution_list = []
    solution_list.append( [  d_3s2,  th_1s1,  th_2s1,  th_4s1,  ] )
    solution_list.append( [  d_3s1,  th_1s2,  th_2s2,  th_4s2,  ] )

    return(solution_list)


#  4x4 transforms which are pure rotations
def RotX4_N(t):
    return(np.matrix([
        [1,         0,           0,      0],
        [0, np.cos(t),  -np.sin(t),      0],
        [0, np.sin(t),   np.cos(t),      0],
        [0,0,0,1.0]
        ]))

def RotY4_N(t):
    return(np.matrix([
        [ np.cos(t),   0,      np.sin(t),    0],
        [0,            1,          0    ,    0],
        [-np.sin(t),   0,      np.cos(t),    0],
        [0,0,0,1]
        ]))

def RotZ4_N(t):
    return(np.matrix([
        [ np.cos(t),  -np.sin(t),       0,    0],
        [ np.sin(t),   np.cos(t),       0,    0],
        [ 0,              0,            1,    0],
        [0,0,0,1]
        ]))
