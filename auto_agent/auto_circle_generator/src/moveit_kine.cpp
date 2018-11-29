/******
 * This file is to calculate the kinematics using MoveIt functions.
 * The main goal is to get the relation of this space frame and the r2_control space frame.
 * Yongming Qin
 * 20180803
*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "Raven_PathPlanner.h"
#include <iostream>
#include <string>
#include <vector>

using namespace std;

string s_eef;

// This function does the forward kinematics and only get the positon (no orientation) of end effector.
vector<double> get_eef_pos(robot_state::RobotState &joint_state)
{
    const Eigen::Affine3d &eef_state = joint_state.getGlobalLinkTransform(s_eef);
    auto temp = eef_state.translation();
    vector<double> temp2;
    temp2.push_back(temp[0]); temp2.push_back(temp[1]); temp2.push_back(temp[2]);
    return temp2;
}

int main(int argc, char **argv) {
    int arm_flag = atoi(argv[1]); // choose which arm. 0/1 for left/right
    string which_arm, s_arm_group;
    if (arm_flag == 0) {
        which_arm = "L";
        s_eef = "wrist_L"; s_arm_group = "left_arm";
    } else if (arm_flag == 1) {
        which_arm = "R";
        s_eef = "wrist_R"; s_arm_group = "right_arm";
    } else {
        cout << "Not valid arm choosing. 0/1 for the second parameter" << endl;
        return 1;
    }
    cout << s_eef << "   " << s_arm_group << endl;


    ros::init(argc, argv, "moveit_kine");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // for moveit functions.
    static const std::string PLANNING_GROUP = s_arm_group;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    robot_state::RobotState tmp_state(*move_group.getCurrentState());

    double joints[3];
    if (argc == 5) { // manually give the joint values
        for (int i=0; i<3; ++i ) joints[i] = stod(argv[i+2]);
        vector<double> vec_joints{joints[0], joints[1], joints[2]}; // -0.45
        // transform joints values to a RobotState
        tmp_state.setJointGroupPositions(joint_model_group, vec_joints);
    }

    // get joint positions
    // http://docs.ros.org/jade/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html
    // shoulder_L, elbow_L, insertion_L, ..._R
    joints[0] = *tmp_state.getJointPositions("shoulder_"+which_arm);
    joints[1] = *tmp_state.getJointPositions("elbow_"+which_arm);
    joints[2] = *tmp_state.getJointPositions("insertion_"+which_arm);
    cout << "joints value: " << joints[0] << " " << joints[1] << " " << joints[2] << endl;

    /* no needed. for future reference
    // transform joints values to a RobotState
    vector<double> cur_jpos;
    tmp_state.setJointGroupPositions(joint_model_group, cur_jpos); */

    vector<double> tmp_eef_pos = get_eef_pos(tmp_state); // 3 elements
    cout << "eef position: " << tmp_eef_pos[0] << " " << tmp_eef_pos[1] << " " << tmp_eef_pos[2] << endl;

    return 0;
}
