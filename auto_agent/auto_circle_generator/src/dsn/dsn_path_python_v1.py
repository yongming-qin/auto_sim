#!/usr/bin/env python

# This file is to publish the produced motion path to a csv file for run.py
# Get the joint space trajectory and extrapolate it.
# Yongming Qin
# 20181112 based on motion_path_publish_v3.cpp
# v2: for dsn paper. I copy the #ifdef simulator to UW codes. This works well with automove topic.
#   Thus I will use the resulting csv file for packetgen.
#   Plan the path for moving from object to container.
#   The result is absolute position.
# dsn_path_python_v1: change the motion space from joint space to cartesian space. Use python.

from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



class MoveGroupInterfaceRaven(object):
    def __init__(self):
        super(MoveGroupInterfaceRaven, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_grou_interace_raven', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "left_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

        planning_frame = group.get_planning_frame()
        print("Reference frame: %s" % planning_frame)

        eef_link = group.get_end_effector_link()
        print("End effector: %s" % eef_link)

        group_names = robot.get_group_names()
        print("Robot Groups: ", group_names)

        print("======== Printing robot states")
        print(robot.get_current_state)
        print(" ")

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        group = self.group
        
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0.522
        joint_goal[1] = 1.577
        joint_goal[2] = -0.05

        group.go(joint_goal, wait=True)
        group.stop()


    def test_ik(self):
        group = self.group

    def go_to_pose_goal(self):
        group = self.group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = -0.12
        pose_goal.position.y = -0.015
        pose_goal.position.z = 0.0
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

    def plan_cartesian_path(self, scale=1):
        group = self.group

        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1
        wpose.position.y += scale * 0.2
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0)
        
        return plan, fraction
    
    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)


def main():
    try:
        print("===== Information...")
        #raw_input()
        practice = MoveGroupInterfaceRaven()

        print("===== pose goal ...")
        #raw_input()
        # practice.go_to_joint_state()
        practice.go_to_pose_goal()




    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
        




if __name__ == "__main__":
    main()
