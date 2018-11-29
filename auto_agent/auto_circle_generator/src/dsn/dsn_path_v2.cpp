/******
 * This file is to publish the produced motion path to a csv file for run.py
 * Get the joint space trajectory and extrapolate it.
 * Yongming Qin
 * 20181112 based on motion_path_publish_v3.cpp
 * v2: for dsn paper. I copy the #ifdef simulator to UW codes. This works well with automove topic.
 *      Thus I will use the resulting csv file for packetgen.
 *      Plan the path for moving from object to container.
 *      The result is absolute position.
 *
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
#include "../Raven_PathPlanner.h"
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;


string s_eef;
// This function does the forward kinematics and only get the positon (no orientation) of end effector.
std::vector<double> get_eef_pos(robot_state::RobotState &joint_state)
{
    const Eigen::Affine3d &eef_state = joint_state.getGlobalLinkTransform(s_eef);
    auto tmp_trans = eef_state.translation();
    auto tmp_rot = eef_state.affine(); // 3x3 rotation matrix
    std::vector<double> ret(3+9);
    for (int i=0; i<3; i++) ret[i] = tmp_trans[i];
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++)  ret[i*3+j+3] = tmp_rot(i,j);
    }
    return ret;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "dsn_path");
    int choice = atoi(argv[1]); // choose motion planning algorithm, not used
    int arm_flag = atoi(argv[2]); // choose which arm. 0/1 for left/right
    const int DOF = 3;
    string s_arm_group;
    if (arm_flag == 0) {
        s_eef = "wrist_L"; s_arm_group = "left_arm";
    } else if (arm_flag == 1) {
        s_eef = "wrist_R"; s_arm_group = "right_arm";
    } else {
        cout << "Not valid arm choosing. 0/1 for the second parameter" << endl;
        return 1;
    }

    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1); // for multi thread. seems not suit for here
    spinner.start();

    /*-------------------------------------Target Objects----------------------------------------*/
    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = s_arm_group;
    //static const std::string PLANNING_GROUP2 = "arms";
    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    //moveit::planning_interface::MoveGroupInterface two_arms_move_group(PLANNING_GROUP2);
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //planning_scene_interface.clearDiffs();
    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr =
            make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    // Raw pointers are frequently used to refer to the  planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //const robot_state::JointModelGroup *two_arms_joint_model_group =
    //    two_arms_move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);


    /*-------------------approach the target position--------------------------------*/
    // start position in joint space, moveit space: insertion
    vector<double> start_jpos = {0.523, 1.577, -0.05}; // homing
    //vector<double> start_jpos = {0.70, 1.22, -0.10}; // aboce object
    const double l_DEG2RAD = 3.14159265/180;
    //vector<double> start_jpos = {(39.74654 * l_DEG2RAD), (67.81275 * l_DEG2RAD), (20.3191 * l_DEG2RAD - 0.45)}; // for dsn, setup

    for (int num=0; num<22; num++)
    {
        std::ofstream table; // for recording
        string file_name = "planned_path//homing_to_object_";
        file_name = file_name + to_string(num) + ".csv";
        table.open(file_name);
        //table.open("object_to_container_1.csv");


        cout << "start_jpos (radian) for " << s_arm_group << ":";
        for (auto v : start_jpos) {
            cout << " " << v;
        } cout << endl;
        robot_state::RobotState start_state(*move_group.getCurrentState());
        start_state.setJointGroupPositions(joint_model_group, start_jpos);
        move_group.setStartState(start_state);

        // goal position in joint space
        robot_state::RobotState goal_state(*move_group.getCurrentState());
        //vector<double> goal_jpos = {0.272, 1.070, 0.342-0.45}; //object, moveit space: insertion
        //vector<double> goal_jpos = {0.4452301263809204, 1.1200997829437256, -0.11196363091468808}; // for dsn

        vector<double> goal_jpos = {0.70, 1.22, -0.15}; // object
        //vector<double> goal_jpos = {0.70, 1.80, -0.15};
        cout << "goal_jops:"; for (auto v : goal_jpos) cout << " " << v; cout << endl;
        goal_state.setJointGroupPositions(joint_model_group, goal_jpos);
        move_group.setJointValueTarget(goal_state);

        /*---------------------------------------set planner--------------------------------------------*/
        // FMTkConfigDefault,  PRMkConfigDefault, RRTkConfigDefault, RRTstarkConfigDefault
#ifdef ARGUMENT_PLANNER
        if (choice == 1) move_group.setPlannerId("FMT");
        else if (choice == 2) move_group.setPlannerId("PRM");
        else if (choice == 3) move_group.setPlannerId("RRT");
        else if (choice == 4) move_group.setPlannerId("RRTstar");
        else {
            cout << "not a recognizable argument" << endl;
            return 0;
        }
#else
        if (num == 0) move_group.setPlannerId("SBL");
        else if (num == 1) move_group.setPlannerId("EST");
        else if (num == 2) move_group.setPlannerId("LBKPIECE");
        else if (num == 3) move_group.setPlannerId("BKPIECE");
        else if (num == 4) move_group.setPlannerId("KPIECE");
        else if (num == 5) move_group.setPlannerId("RRT");
        else if (num == 6) move_group.setPlannerId("RRTConnect");
        else if (num == 7) move_group.setPlannerId("RRTstar");
        else if (num == 8) move_group.setPlannerId("TRRT");
        else if (num == 9) move_group.setPlannerId("PRM");
        else if (num == 10) move_group.setPlannerId("PRMstar");
        else if (num == 11) move_group.setPlannerId("FMT");
        else if (num == 12) move_group.setPlannerId("BFMT");
        else if (num == 13) move_group.setPlannerId("PDST");
        else if (num == 14) move_group.setPlannerId("STRIDE");
        else if (num == 15) move_group.setPlannerId("BiTRRT");
        else if (num == 16) move_group.setPlannerId("LBTRRT");
        else if (num == 17) move_group.setPlannerId("BiEST");
        else if (num == 18) move_group.setPlannerId("ProjEST");
        else if (num == 19) move_group.setPlannerId("LazyPRM");
        else if (num == 20) move_group.setPlannerId("LazyPRMstar");
        else if (num == 21) move_group.setPlannerId("SPARS");
        else if (num == 22) move_group.setPlannerId("SPARStwo");
        else {cout << "Error. Planner not found." << endl;}
#endif

        /*-----------------------Call the planner to compute the plan and visualize it---------------------*/
        // Note that we are just planning, not asking move_group to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        /*---------------------------------Plan--------------------------------------*/
        moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
        std::cout << "The procession time of FMT is: " << my_plan.planning_time_ << std::endl;

        /*--------------------------------Process my_plan-------------------------------*/
        

        //Do forward kinematics. Calculate transformation increments
        robot_state::RobotState tmp_state(*move_group.getCurrentState());
        std::vector<double> moveit_eef_pos;
        std::vector<std::vector<double>> path_from_moveit;

        // Calculate the eff position.
        vector<double> moveit_pose, raven_pose(3,0);
        for (auto point : my_plan.trajectory_.joint_trajectory.points) {
            double tmp_pos[DOF];
            for (int i=0; i<DOF; i++) tmp_pos[i] = point.positions[i];
            // transform joints values to a RobotState
            tmp_state.setJointGroupPositions(joint_model_group, tmp_pos);
            moveit_pose = get_eef_pos(tmp_state); // 3 elements
            // moveit_eef_pos.push_back(joint.time_from_start.toNSec()); // 4th elements

            // !1from moveit space to raven space
            
            raven_pose[0] = moveit_pose[2];
            raven_pose[1] = moveit_pose[1];
            raven_pose[2] = -moveit_pose[0] - 0.2;
            path_from_moveit.push_back(raven_pose);
            for (int i=0; i<2; ++i) {
                table << raven_pose[i] * 1000 * 1000 << ",";
                cout << raven_pose[i] * 1000 * 1000 << " ";
            }
            table << raven_pose[2] * 1000 * 1000 << endl;
            cout << raven_pose[2] * 1000 * 1000 << endl;
        }
        table <<"eof,," << endl;
        table.close();
    }
    



    
        
#ifdef NEXT
        vector<vector<double>> raven_path_pos;
        for (auto pos : path_from_moveit) {
            raven_path_pos.push_back(vector<double>{-0.2 - pos[2], pos[1], pos[0], pos[3]});
        }
        for (auto pos : raven_path_pos) {
            cout << "eef_pos(raven space):"; for (int i=0; i<4; ++i) cout << "\t" << pos[i]; cout << endl;
        } cout << endl;




        /* for reference
        std::vector<std::vector<double>> path_pos_incr;
        std::vector<double> tmp_v{0, 0, 0, 0};
        path_pos_incr.push_back(tmp_v);
        for (auto it=path_pos.begin(); it != path_pos.end()-1; ++it) {
            for (decltype(tmp_v.size()) i=0; i<4; ++i) tmp_v[i] = ( (*(it+1))[i]-(*it)[i] );
            path_pos_incr.push_back(tmp_v);
            cout << "tmp_v: " << tmp_v[0] << ", " << tmp_v[1] << ", " << tmp_v[2] << ", " << tmp_v[3] << endl;
        } */



        /*-----------------------Publish the planned path to raven_automove topic--------------*/

        geometry_msgs::Vector3 delta_pos_zero; // no position increment
        delta_pos_zero.x=0; delta_pos_zero.y=0; delta_pos_zero.z=0; 
        geometry_msgs::Quaternion delta_ori_zero; // no rotation increment
        delta_ori_zero.x=0; delta_ori_zero.y=0; delta_ori_zero.z=0; delta_ori_zero.w=1;
        geometry_msgs::Vector3 delta_pos; //
        // 0, 1 for left, right
        automove_msg.tf_incr[0].translation = delta_pos_zero;
        automove_msg.tf_incr[0].rotation = delta_ori_zero;
        automove_msg.tf_incr[1].translation = delta_pos_zero;
        automove_msg.tf_incr[1].rotation = delta_ori_zero;
        // assume at least one element.



        cout << "Start publish? y?" << endl;
        while (cin >> in_s) {
            if (in_s == "y") {
                break;
            } else if (in_s == "e") {
                return 0;
            } else {
                cout << "Please type \"y\" if you want to start publish." << endl;
            }
        }

        /* for future use
        ros::Time time_start = ros::Time::now();
        ros::Time time_cur;
        (time_cur-time_start).toNSec() */

        const long m2um = 1000 * 1000;
        int n_points = 0;
        for (auto it=path_from_moveit.cbegin(); it != path_from_moveit.cend(); ++it)
        {
            cout << n_points << endl;
            n_points++;
            cout << "current waypoint: " << (*it)[0]*m2um << " " << (*it)[1]*m2um << " " << (*it)[2]*m2um << endl;
        
            automove_msg.hdr.stamp = automove_msg.hdr.stamp.now(); //?

            // every step, move 1um delta_pos (from experiment)
            int act_pos_d[3], incr[3];
            for (int i=0; i<3; ++i) act_pos_d[i] = gold_s.pos_d[i];
            cout << "actual eef position: " << act_pos_d[0] << " " << act_pos_d[1] << " " << act_pos_d[2] << endl;

            long step = 0;
            int idx = 0;

            incr[0] = 1; // make sure go into the below loop at first time
            while (incr[0] != 0 || incr[1] != 0 || incr[2] != 0)
            {
                step++;
                for (int i=0; i<3; ++i)
                {
                    // approaching method till very close
                    int dif = (*it)[i]*m2um - act_pos_d[i]; // difference
                    int range = 3;
                    if (abs(dif) < range) {
                        incr[i] = dif;
                    } else if (abs(dif) == 0) {
                        incr[i] = 0;
                    } else {
                        incr[i] = (dif > 0) ? 2 : -2;
                    }
                    act_pos_d[i] += incr[i];
                }


                // Be careful of the relation. I got the relation by testing.
                if (arm_flag == 0) { // left arm
                    delta_pos.x=incr[2]; delta_pos.y=incr[1]; delta_pos.z=-incr[0];
                } else if (arm_flag == 1) { // right arm
                    delta_pos.x=incr[2]; delta_pos.y=-incr[1]; delta_pos.z=incr[0];
                }


                automove_msg.tf_incr[arm_flag].translation = delta_pos; // arm_flag, 0/1 is for left/right arm

                // cout info
                if (step % 10 == 0) {
                    idx++;
                    cout << "idx of sequence: " << idx << endl;
                    cout << "incr of moveit space: " << incr[0] << " " << incr[1] << " " << incr[2] << endl;
                    cout << "delta_pos for raven: " << delta_pos << endl;
                    /*
                    cout << "jpos: " << gold_s.jpos[arm_flag*3] << " " << gold_s.jpos[1+arm_flag*3] << " " << gold_s.jpos[2+arm_flag*3] << endl;
                    // calculate eef position
                    vector<double> cur_jpos;
                    for (int i=0; i<3; ++i) {
                        cur_jpos.push_back(gold_s.jpos[i+arm_flag*3]); //!! left/right arm
                    }
                    // transform joints values to a RobotState
                    tmp_state.setJointGroupPositions(joint_model_group, cur_jpos);
                    moveit_eef_pos = get_eef_pos(tmp_state); // 3 elements
                    cout << "eef position: " << moveit_eef_pos[0] << " " << moveit_eef_pos[1] << " " << moveit_eef_pos[2] << endl; */

                }
            }
                
            
        }
    }

#endif
    //table.close();

    //ros::Duration(5.0).sleep();
    //ros::shutdown();
    return 0;
}


