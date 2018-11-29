/******
 * This file is to publish the produced motion path to raven topic raven_automove.
 * Then raven robots will receive the desired cartesian positions and move.
 * Yongming Qin
 * 20180321 // probably only start the coding
 * 2018517: The problem is this is a open loop. After publishing, there is a difference between real robots and what we expected.
 * 20180805: v2. Make it a closed loop. The **desired** (not actual) eff position is transimitted back
 *           and compared with each point of the planned path.
 *           Untill the robot reach the point within a range, next point is published.
 * 20180806: v2: Start position and goal postion are set manually. (homing, then move up)
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
#include "Raven_PathPlanner.h"
#include <iostream>
#include <string>

using namespace std;

struct state {
    double jpos[6]; // the first three joints of left and right arms
    int pos[6]; // um
    int pos_d[6]; // um
    float ori[18];
    float ori_d[18];
};
string s_eef;


// This function does the forward kinematics and only get the positon (no orientation) of end effector.
std::vector<double> get_eef_pos(robot_state::RobotState &joint_state)
{
    const Eigen::Affine3d &eef_state = joint_state.getGlobalLinkTransform(s_eef);
    auto temp = eef_state.translation();
    std::vector<double> temp2;
    temp2.push_back(temp[0]); temp2.push_back(temp[1]); temp2.push_back(temp[2]);
    return temp2;
}

// The callback function for the subscriber
state gold_s;

void ravenstate_callback(const raven_state msg) {
    for (int arm = 0; arm < 2; arm++) {
        for (int i=0; i<3; i++) { // there are 8x2 elements. The fourth one is not used.
            //gold_s.jpos[i+arm*3] = msg.jpos[i+arm*8] * 3.14159 / 180; // in r2_control. The first three joints multiply ...
            gold_s.jpos[i+arm*3] = msg.jpos[i+arm*8];
            if (i==2) {
                gold_s.jpos[i+arm*3] = gold_s.jpos[i+arm*3] - 0.45; // based on the relation between urdf and r2_control
                continue;
            }
        }

        // for future
        for (int i=0; i<9; i++) {
            gold_s.ori[i+arm*9] = msg.ori[i+arm*9];
            gold_s.ori_d[i+arm*9] = msg.ori_d[i+arm*9];
        }
    }

    // from raven space to moveit space
    gold_s.pos[0] = -200000-msg.pos[2];
    gold_s.pos[1] = msg.pos[1];
    gold_s.pos[2] = msg.pos[0];
    gold_s.pos_d[0] = -200000-msg.pos_d[2];
    gold_s.pos_d[1] = msg.pos_d[1];
    gold_s.pos_d[2] = msg.pos_d[0];
}

double trunc(double in) {
    if (in < -1.5) {in = -1.5;}
    else if (in > 1.5) {in = 1.5;}
    return in;
}


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "auto_deb");
    int choice = atoi(argv[1]); // choose motion planning algorithm
    int arm_flag = atoi(argv[2]); // choose which arm. 0/1 for left/right
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
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::ofstream table; // for recording
    table.open("ana.txt");

    /*-------------------Subscribe raven_state topic. Get the starting position--------------*/
    ros::Subscriber sub_ravenstate = n.subscribe("ravenstate", 100, ravenstate_callback);

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
    // untill the actural eef is within a range of the desired position
    int flag = 1;
    while (flag) {
        flag = 0;
        // Subscribe for a period so that the value is updated. Then get the current raven state.
        ros::Time t;
        t = t.now();
        while (ros::ok() && (t.now() - t).toSec() <= 1) {
            ros::spinOnce();
        }

        vector<double> start_jpos;
        for (int i=0; i<3; ++i) {
            start_jpos.push_back(gold_s.jpos[i+arm_flag*3]); //!! left/right arm
        }
        cout << "start_jpos for " << s_arm_group << "(degree): " << start_jpos[0] << ", " << start_jpos[1] << ", " << start_jpos[2] << endl;

        //monitor_ptr->clearOctomap();
        //ros::Duration(5.0).sleep();
        //cout << "Octomap refreshed." << endl;

        /*-----------------------Avoid collision--------------------------------------*/
        // Set the starting and goal positions, plan to the new joint space goal.
        robot_state::RobotState start_state(*move_group.getCurrentState());
        start_jpos[0] = 0.523; start_jpos[1] = 1.577; start_jpos[2] = -0.05;
        start_state.setJointGroupPositions(joint_model_group, start_jpos);
        move_group.setStartState(start_state);


        // wait for the user choosing the goal position
        string in_s;
        cout << "Please set the goal position in Moveit. The goal position will be the current state." << endl;
        cout << "Then type \"p\" " << endl;
        while (cin >> in_s) {
            if (in_s == "p") {
                break;
            } else if (in_s == "e") {
                return 0;
            } else {
                cout << "Please type \"p\" after setting the goal positon." << endl;
            }
        }
        
        robot_state::RobotState goal_state(*move_group.getCurrentState());
        vector<double> goal_jpos = {0.523, 1.577, 0.05};
        goal_state.setJointGroupPositions(joint_model_group, goal_jpos);
        move_group.setJointValueTarget(goal_state);

        /*---------------------------------------set planner--------------------------------------------*/
        // FMTkConfigDefault,  PRMkConfigDefault, RRTkConfigDefault, RRTstarkConfigDefault
        if (choice == 1) move_group.setPlannerId("FMTkConfigDefault");
        else if (choice == 2) move_group.setPlannerId("PRMkConfigDefault");
        else if (choice == 3) move_group.setPlannerId("RRTkConfigDefault");
        else if (choice == 4) move_group.setPlannerId("RRTstarkConfigDefault");
        else {
            cout << "not a recognizable argument" << endl;
            return 0;
        }

        /*-----------------------Call the planner to compute the plan and visualize it---------------------*/
        // Note that we are just planning, not asking move_group to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        /*---------------------------------Plan--------------------------------------*/
        moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
        ROS_INFO_NAMED("Independent study FMT %s", success ? "SUCCESSFUL" : "FAILED");
        std::cout << "The procession time of FMT is: " << my_plan.planning_time_ << std::endl;
        //ROS_INFO_NAMED("Independent study", "The first element is %f", my_plan.trajectory_.joint_trajectory.points[0].positions[0]);
        /*--------------------------------Process my_plan-------------------------------*/
        //Do forward kinematics. Calculate transformation increments
        robot_state::RobotState tmp_state(*move_group.getCurrentState());
        std::vector<double> tmp_eef_pos;
        std::vector<std::vector<double>> path_pos;

        for (auto &joint : my_plan.trajectory_.joint_trajectory.points) {
            auto tmp_pos = joint.positions;
            cout << "joint sequence: " << tmp_pos[0] << ", " << tmp_pos[1] << ", " << tmp_pos[2] << endl;
        }

        // Calculate the eff position.
        for (auto &joint : my_plan.trajectory_.joint_trajectory.points) {
            auto tmp_pos = joint.positions;
            // transform joints values to a RobotState
            tmp_state.setJointGroupPositions(joint_model_group, tmp_pos);
            tmp_eef_pos = get_eef_pos(tmp_state); // 3 elements
            tmp_eef_pos.push_back(joint.time_from_start.toNSec()); // 4 elements
            path_pos.push_back(tmp_eef_pos);
            cout << "tmp_eef_pos: " << tmp_eef_pos[0] << ", " << tmp_eef_pos[1] << ", "
                << tmp_eef_pos[2] << ", " << tmp_eef_pos[3] << endl;
        }




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
        ros::Publisher pub_motion = n.advertise<raven_automove>("raven_automove", 1);
        ros::Rate loop_rate1(1000); // for below publisher
        raven_automove automove_msg;

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
        for (auto it=path_pos.cbegin(); it != path_pos.cend(); ++it)
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
                        incr[i] = (dif > 0) ? 1 : -1;
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
                if (step % 100 == 0) {
                    idx++;
                    cout << "idx of sequence: " << idx << endl;
                    cout << "incr: " << incr[0] << " " << incr[1] << " " << incr[2] << endl;
                    cout << "delta_pos: " << delta_pos << endl;
                    /*
                    cout << "jpos: " << gold_s.jpos[arm_flag*3] << " " << gold_s.jpos[1+arm_flag*3] << " " << gold_s.jpos[2+arm_flag*3] << endl;
                    // calculate eef position
                    vector<double> cur_jpos;
                    for (int i=0; i<3; ++i) {
                        cur_jpos.push_back(gold_s.jpos[i+arm_flag*3]); //!! left/right arm
                    }
                    // transform joints values to a RobotState
                    tmp_state.setJointGroupPositions(joint_model_group, cur_jpos);
                    tmp_eef_pos = get_eef_pos(tmp_state); // 3 elements
                    cout << "eef position: " << tmp_eef_pos[0] << " " << tmp_eef_pos[1] << " " << tmp_eef_pos[2] << endl; */

                }
                
                pub_motion.publish(automove_msg);
                ros::spinOnce();
                loop_rate1.sleep();
            }
                
            
        }
        ros::Duration(5.0).sleep(); // wait till the arm stops movement.
    }

    
    
#ifdef ANALYSIS    
    /*--------------------------Analysis---------------------------------------------------*/
    robot_state::RobotState temp_state(*move_group.getCurrentState());
    int step = 0;
    double length = 0;
    std::vector<double> max_velocities(3, 0);
    std::vector<double> max_accelerations(3, 0);
    std::vector<double> pre_eef_state;
    std::vector<double> temp_eef_state;
    for (auto &joint : my_plan1.trajectory_.joint_trajectory.points) {
        auto temp_positions = joint.positions;
        temp_state.setJointGroupPositions(joint_model_group, temp_positions);
        temp_eef_state = eef_state(temp_state);
        if (step == 0) {
        pre_eef_state = eef_state(temp_state);
        }
        step++;
        // length of the trajectory
        length += sqrt(pow(temp_eef_state[0] - pre_eef_state[0], 2)
                        + pow(temp_eef_state[1] - pre_eef_state[1], 2)
                        + pow(temp_eef_state[2] - pre_eef_state[2], 2));
        pre_eef_state = temp_eef_state;
        /* max_velocities, max_accelerations */
        for (int i=0; i<3; i++) {
        if ( std::abs(joint.velocities[i]) > std::abs(max_velocities[i]) ) {
            max_velocities[i] = joint.velocities[i];
        }
        if ( std::abs(joint.accelerations[i]) > std::abs(max_accelerations[i]) ) {
            max_accelerations[i] = joint.accelerations[i];
        }
        }
    }
    std::cout << "path length:" << length << std::endl;
    std::cout << "max velocity of each joint:"
        << max_velocities[0] << ", " << max_velocities[1] << ", " << max_velocities[2] << std::endl;
    std::cout << "max accelearation of each joint:"
        << max_accelerations[0] << ", " << max_accelerations[1] << ", " << max_accelerations[2] << std::endl;

    /* todo std::cout << my_plan.trajectory_.multi_dof_joint_trajectory.points[20].transforms[0].translation.x;
    std::cout << my_plan.trajectory_.multi_dof_joint_trajectory.joint_names[0] << std::endl; */

    table << my_plan1.planning_time_ << "&" << length << "&";
    table << "[" << max_velocities[0] << ", " << max_velocities[1] << ", " << max_velocities[2] << "]" << "&";
    table << "[" <<max_accelerations[0] << ", " << max_accelerations[1] << ", " << max_accelerations[2] << "]" << std::endl;

#endif


    table.close();

    //ros::Duration(5.0).sleep();
    //ros::shutdown();
    return 0;
}

