/******
 * This file is to publish values to raven topic raven_automove.
 * The main goal is for testing.
 * Then raven robots will receive the desired cartesian positions and move.
 * Yongming Qin
 * 20180517: test the relation of delta_pos and the actual movement of the robot
 * 20180807: This version is not pure. It includes partes of the motion planning. Create v2 and prune it.
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

using namespace std;
struct state {
    double jpos[6]; // the first three joints of left and right arms

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
        for(int i=0; i<3; i++) { // there are 8x2 elements. The fourth one is not used.
            //gold_s.jpos[i+arm*3] = msg.jpos[i+arm*8] * 3.14159 / 180; // in r2_control. The first three joints multiply ...
            gold_s.jpos[i+arm*3] = msg.jpos[i+arm*8];
            if (i==2) {
                gold_s.jpos[i+arm*3] = gold_s.jpos[i+arm*3] - 0.45; // based on the relation between urdf and r2_control
                continue;
            }
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move");
    int arm_flag = atoi(argv[1]); // choose which arm. 0/1 for left/right
    // increment values for x,y,z. 2 is a little quick. set as 1.5
    double inc_x = atof(argv[2]); // choose motion planning algorithm
    double inc_y = atof(argv[3]); // choose motion planning algorithm
    double inc_z = atof(argv[4]); // choose motion planning algorithm
    cout << "increment value for x,y,z: " << inc_x << " " << inc_y << " " << inc_z << endl;
    string s_arm_group;
    if (arm_flag == 0) {
        s_eef = "wrist_L"; s_arm_group = "larm";
    } else if (arm_flag == 1) {
        s_eef = "wrist_R"; s_arm_group = "rarm";
    } else {
        cout << "Not valid arm choosing. 0/1 for the second parameter" << endl;
        return 1;
    }
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*-------------------Subscribe raven_state topic. Get the starting position--------------*/
    ros::Subscriber sub_ravenstate = n.subscribe("ravenstate", 100, ravenstate_callback);

    /*-----------------------Publish the planned path to raven_automove topic--------------*/
    ros::Publisher pub_motion = n.advertise<raven_automove>("raven_automove", 20);
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

    // for moveit functions.
    static const std::string PLANNING_GROUP = s_arm_group;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    robot_state::RobotState tmp_state(*move_group.getCurrentState());
    std::vector<double> tmp_eef_pos, pre_eef_pos, cur_jpos;
    for (int i=0; i<3;++i) {pre_eef_pos.push_back(0); cur_jpos.push_back(0);}
    int step = 0; // use number of steps to calculate time
    
    while (ros::ok()) {
        automove_msg.hdr.stamp = automove_msg.hdr.stamp.now(); //?
        
        // Be careful of the relation. I got the relation by testing.
        //delta_pos.x=tmp_pos_incr[2]*1000;delta_pos.y=-tmp_pos_incr[1]*1000;delta_pos.z=tmp_pos_incr[0]*1000;
        delta_pos.x=inc_x;delta_pos.y=inc_y;delta_pos.z=inc_z; // for testing
        
        step++;
        automove_msg.tf_incr[arm_flag].translation = delta_pos; // arm_flag, 0/1 is for left/right arm
        
        if (step % 100 == 0) {
            cout << s_arm_group << endl;
            cout << "jpos: " << gold_s.jpos[arm_flag*3] << " " << gold_s.jpos[1+arm_flag*3] << " " << gold_s.jpos[2+arm_flag*3] << endl;
            // calculate eef position
            for (int i=0; i<3; ++i) {
                cur_jpos[i] = gold_s.jpos[i+arm_flag*3]; //!! left/right arm
            }
            // transform joints values to a RobotState
            tmp_state.setJointGroupPositions(joint_model_group, cur_jpos);
            tmp_eef_pos = get_eef_pos(tmp_state); // 3 elements
            cout << "eef position: " << tmp_eef_pos[0] << " " << tmp_eef_pos[1] << " " << tmp_eef_pos[2] << endl;
            //cout << "difference: " << (tmp_eef_pos[0]-pre_eef_pos[0])*1000 << " " << (tmp_eef_pos[1]-pre_eef_pos[1])*1000 << " "
                //<< (tmp_eef_pos[2]-pre_eef_pos[2])*1000 << endl;
            for (int i=0; i<3;++i) {pre_eef_pos[i] = tmp_eef_pos[i];}
        }
        
        pub_motion.publish(automove_msg);
        ros::spinOnce();
        loop_rate1.sleep();
    }

}
