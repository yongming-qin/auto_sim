/**
 * This class includes the members and member functions needed for implementing autonomous debridement.
 * Yongming Qin
 * 20180807
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include "Raven_PathPlanner.h" // melody
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

struct state {
    double jpos[6]; // the first three joints of left and right arms
    int pos[6]; // um
    int pos_d[6]; // um
    float ori[18];
    float ori_d[18];
};

std::vector<double> get_eef_pos(robot_state::RobotState &joint_state);

class Auto_deb
{
    using namespace std; // Toch
    Auto_deb() = default;
    Auto_deb(const string &s1, const string &s2): PLANNING_GROUP(s1), s_eef(s2)
    {
        move_group(PLANNING_GROUP);
        monitor_ptr = make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    }
    





    int move_to()



    string s_eef;
    /*-------------------------------------Target Objects----------------------------------------*/
    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const string PLANNING_GROUP;
    //static const std::string PLANNING_GROUP2 = "arms";
    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group;
    //moveit::planning_interface::MoveGroupInterface two_arms_move_group(PLANNING_GROUP2);
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //planning_scene_interface.clearDiffs();
    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr;

    // Raw pointers are frequently used to refer to the  planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group;
    //const robot_state::JointModelGroup *two_arms_joint_model_group =
    //    two_arms_move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);





}



// This function does the forward kinematics and only get the positon (no orientation) of end effector.
std::vector<double> get_eef_pos(robot_state::RobotState &joint_state, const string s_eef)
{
    const Eigen::Affine3d &eef_state = joint_state.getGlobalLinkTransform(s_eef);
    auto temp = eef_state.translation();
    std::vector<double> temp2;
    temp2.push_back(temp[0]); temp2.push_back(temp[1]); temp2.push_back(temp[2]);
    return temp2;
}