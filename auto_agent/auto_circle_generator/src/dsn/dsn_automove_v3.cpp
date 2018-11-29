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
 * 20180806: v3: !! set a pose goal of cartesian space instead of joint space
 * 20180807: v3: The analysis part is deleted. If you want to find this part to analyze the path, go to preious version.
 * 20180823: v5: based on v3. Move to deried position directly without motion planning. input raven space position.
 * 
 * dsn_automove_v2: 2018/11/15 Read the trajectory produced from moveit
 * dsn_automove_v3: 2018/11/18 The movements of Homing position to the position above the object
 *                   and the object's position to the position above the container are
 *                  all produced by motion planning algorithm.
 * 
 *
*/

#include <ros/ros.h>
#include "../Raven_PathPlanner.h"
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <limits>

#include <raven_2/SegLabel.h>

#include <pc_analysis/obj_pos.h>


using namespace std;
using namespace Eigen;

struct state
{
    double jpos[6]; // the first three joints of left and right arms
    int pos[6];     // um
    int pos_d[6];   // um
    float ori[18];
    float ori_d[18];
};
string s_eef;

struct Seg_change
{ // representation of the change of each segment

    Seg_change() {
        zero_tl.x = 0; zero_tl.y = 0; zero_tl.z = 0;
        id_ori.x = 0; id_ori.y = 0; id_ori.z = 0; id_ori.w = 1;
        tf.translation = zero_tl; tf.rotation = id_ori;
    }
    void reset()
    {
        b_tl = false;
        b_rot = false;
        tf.translation = zero_tl;
        tf.rotation = id_ori;
        grasp = 0;
        duration = FLT_MAX;
    }
    void keep_tl_reset()
    { // tf.translation does not change
        b_tl = false;
        b_rot = false;
        tf.rotation = id_ori;
        grasp = 0;
        duration = FLT_MAX;
    }

    void info() const {
        cout << "tf.translation: " << tf.translation.x << " " << tf.translation.y << " " << tf.translation.z << endl;
        cout << "tf.rotation: " << tf.rotation.x << " " << tf.rotation.y << " "
            << tf.rotation.z << " " << tf.rotation.w << endl;
        cout << "start translation: " << (b_tl ? "yes" : "no") << endl;
        cout << "start rotation: " << (b_rot ? "yes" : "no") << endl;
        cout << "grasp, duration: " << grasp << " " << duration << endl;
    }


    // use bool for translation and rotation as they are not all target or accumulation
    bool b_tl = false;           // true start translation
    bool b_rot = false;          // true start rotation.
    geometry_msgs::Transform tf; // translation: the target position; rotation: direct accumulation effect
    int grasp = 0;    // left: + close
    float duration = FLT_MAX; // no limits
    int seg_label = -1;
    //
    geometry_msgs::Vector3 zero_tl; // no position increment
    //
    geometry_msgs::Quaternion id_ori; // no rotation increment

};

// The callback function for the subscriber
state gold_s;
void ravenstate_callback(const raven_state msg)
{
    for (int arm = 0; arm < 2; arm++)
    {
        for (int i = 0; i < 3; i++)
        { // there are 8x2 elements. The fourth one is not used.
            // From /raven_state of r2_control, the first three joints are of degrees.
            // !! It is wrong for the thrid joint which is prismatic.
            gold_s.jpos[i + arm * 3] = msg.jpos[i + arm * 8] * 3.14159 / 180;
        }
        //gold_s.jpos[2+arm*3] -= 0.45; // insertion prismatic joint. based on the relation between urdf and r2_control

        // for future
        for (int i = 0; i < 9; i++)
        {
            gold_s.ori[i + arm * 9] = msg.ori[i + arm * 9];
            gold_s.ori_d[i + arm * 9] = msg.ori_d[i + arm * 9];
        }
    }

    //cout << "jpos:"; for (int i=0; i<3; ++i) cout << " " << gold_s.jpos[0]; cout << endl;
    //cout << "pos:"; for (int i=0; i<3; ++i) cout << " " << msg.pos[i]; cout << endl;

    for (int i = 0; i < 3; ++i)
    {
        gold_s.pos[i] = msg.pos[i];
        gold_s.pos_d[i] = msg.pos_d[i];
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_deb");
    int arm_flag = 0; // left arm

    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    /*-------------------Subscribe raven_state topic. Get the starting position--------------*/
    ros::Subscriber sub_ravenstate = n.subscribe("ravenstate", 10, ravenstate_callback);
    ros::Rate loop_rate_sub(100); // subscrbe to get the actual position

    /*-----------------------Publish the planned path to raven_automove topic--------------*/
    ros::Publisher pub_motion = n.advertise<raven_automove>("raven_automove", 1);
    ros::Rate loop_rate_pub(1000); // for below publisher
    raven_automove automove_msg;

    /*-----------------------Service------------------------------------------------------*/
    ros::ServiceClient client = n.serviceClient<raven_2::SegLabel>("add_seg_label");
    raven_2::SegLabel srv;

    Seg_change change;
    // 0, 1 for left, right
    automove_msg.tf_incr[0].translation = change.zero_tl;
    automove_msg.tf_incr[0].rotation = change.id_ori;
    automove_msg.tf_incr[1].translation = change.zero_tl;
    automove_msg.tf_incr[1].rotation = change.id_ori;

    geometry_msgs::Vector3 delta_pos; //

    
    /*-------------------approach the target position--------------------------------*/
    // untill the actural eef is within a range of the desired position
    vector<Seg_change> path;

    if (true) // for future
    {
        // Subscribe for a period so that the value is updated. Then get the current raven state.
        ros::Time t;
        t = t.now();
        while (ros::ok() && ( (t.now() - t).toSec() <= 2 ) )
        {
            ros::spinOnce();
            loop_rate_sub.sleep();
        }

        //vector<int> obj_pos = {-77830, -20349, 13885}; // object's position
        vector<int> obj_pos = {-117925, -48234+1000, -15048};
        if (stoi(argv[1]) != -1) {
            int num_inFile = stoi(argv[1]);
            
            // Open the grasper.
            change.keep_tl_reset();
            change.grasp = -1; // 1-5
            change.duration = 1;
            change.seg_label = -1;
            path.push_back(change);
        /* --------------------------------------------------- */
            // move to the above of the object
            std::ifstream inFile1; // for recording
            inFile1.open("/home/yq/Simulator/mine_ws/data/homing_to_object_"+to_string(num_inFile)+".csv");
            if (!inFile1) {
                cout << "Unable to open file" << endl;
                return 1;
            }

            int val = 0;
            string inStr;
            // the last one may be 0 for x
            while (getline(inFile1, inStr, ',')) {
                if (inStr != "\n") val = int(stof(inStr));
                else break;
                //cout << "raven space xyz: " << val;
                change.keep_tl_reset();
                change.tf.translation.x = val;
                getline(inFile1, inStr, ',');
                if (inStr != "\n") val = int(stof(inStr));
                else break;
                change.tf.translation.y = val;
                cout << " " << val;
                getline(inFile1, inStr, ',');
                if (inStr != "\n") val = int(stof(inStr));
                else break;
                change.tf.translation.z = val;
                cout << " " << val << endl;
                change.b_tl = true;
                change.seg_label = 1;
                path.push_back(change);
            }
            inFile1.close();


#ifdef PREAPPROACHING
            // Approaching the object.
            change.keep_tl_reset();
            change.tf.translation.x = obj_pos[0];
            change.tf.translation.y = obj_pos[1];
            change.tf.translation.z = obj_pos[2];
            change.b_tl = true;
            path.push_back(change);
#endif
            // move a little down
            change.tf.translation.x -= 10000;
            change.seg_label = 1;
            path.push_back(change);

            // Close the Grasper. raven_tf not change
            change.keep_tl_reset();
            change.grasp = 2; // 1-5
            change.duration = 1;
            change.seg_label = 2;
            path.push_back(change);

            
            // move a little up
            change.keep_tl_reset();
            change.tf.translation.x += 40000; // 3cm
            change.b_tl = true;
            change.seg_label = 3;
            path.push_back(change);

    //#define MOVETOCONTAINERMOTION
    #ifdef MOVETOCONTAINERMOTION  
        /* --------------------------------------------------- */   
            // move to the above of the container
            std::ifstream inFile2; // for recording
            inFile2.open("/home/yq/Simulator/mine_ws/data/object_to_container.csv");
            if (!inFile2) {
                cout << "Unable to open file" << endl;
                return 1;
            }

            // the last one may be 0 for x
            while (getline(inFile2, inStr, ',')) {
                if (inStr != "\n") val = int(stof(inStr));
                else break;
                cout << "raven space xyz: " << val;
                change.keep_tl_reset();
                change.tf.translation.x = val;
                getline(inFile2, inStr, ',');
                if (inStr != "\n") val = int(stof(inStr));
                else break;
                change.tf.translation.y = val;
                cout << " " << val;
                getline(inFile2, inStr, ',');
                if (inStr != "\n") val = int(stof(inStr));
                else break;
                change.tf.translation.z = val;
                cout << " " << val << endl;
                change.b_tl = true;
                change.seg_label = 4;
                path.push_back(change);
            }
            inFile2.close();
    #endif

   
            /* // move a little down
            change.tf.translation.x -= 10000;
            path.push_back(change); */

    #define MOVETOCONTAINERFIXED
    #ifdef MOVETOCONTAINERFIXED
            // move to the position above the container
            change.reset();
            change.tf.translation.x = -120000;
            change.tf.translation.y = -15530;
            change.tf.translation.z = 60000;
            change.b_tl = true;
            change.seg_label = 4;
            path.push_back(change);
    #endif
            // open the grasper
            change.keep_tl_reset();
            change.grasp = -1;
            change.duration = 3;
            change.seg_label = 5;
            path.push_back(change);

    #ifdef BACKTOHOMING
            // move a little up
            change.keep_tl_reset();
            change.tf.translation.x += 30000; // 3cm
            change.b_tl = true;
            path.push_back(change);
            // go back to homing position
            change.reset();
            change.tf.translation.x = -77830;
            change.tf.translation.y = -24349;
            change.tf.translation.z = 13885;
            change.b_tl = true;
            path.push_back(change);
    #endif       
            cout << "The number of segments: " << path.size() << endl;
        }
        else {
            // Approaching the given target
            //vector<int> target = {-77830, -20349, 13885};
            vector<int> target = {-150439, -49010, -15411};
            change.tf.translation.x = target[0];
            change.tf.translation.y = target[1];
            change.tf.translation.z = target[2];
            change.b_tl = true;
            path.push_back(change);
            cout << "The number of segments(1): " << path.size() << endl;
        }
        
        cout << "number of segments: " << path.size() << endl;
        for (auto p : path) { p.info(); cout << endl; }
        
//#define CONFIRM
#ifdef CONFIRM
        string in_s;
        cout << "Start this segment? y?" << endl;
        while (cin >> in_s)
        {
            if (in_s == "y") {break;}
            else if (in_s == "e") {return 0;}
            else {cout << "Please type \"y\" if you want to start ." << endl;}
        }
#endif

        const int RANGE = 8;
        const int V = 2;

        int n_seg = 0;
        for (auto it = path.cbegin(); it != path.cend(); ++it)
        {
            
            if (ros::ok()) {
                cout << "The segment: " << n_seg << endl;
                n_seg++;
                it->info();
                cout << endl;
                cout << "current position: "; for (int i=0; i<3; i++) cout << gold_s.pos_d[i] << " "; cout << endl;
                //ros::Duration(2.0).sleep();

#ifdef CONFIRM
                cout << "Start this segment? y?" << endl;
                while (cin >> in_s)
                {
                    if (in_s == "y") {break;}
                    else if (in_s == "e") {return 0;}
                    else {cout << "Please type \"y\" if you want to start ." << endl;}
                }
#endif
                // label is sent to r2_control only at the start of this segment
                srv.request.label_from_auto_agent = it->seg_label;
                client.call(srv);


                int act_pos_d[3];
                for (int i = 0; i < 3; ++i) act_pos_d[i] = gold_s.pos_d[i];
                //cout << "actual eef position: " << act_pos_d[0] << " " << act_pos_d[1] << " " << act_pos_d[2] << endl;

                t = t.now();
                while ( ros::ok() &&
                ( (t.now() - t).toSec() <= it->duration )
                )
                {
                    // reset to zero in case error.
                    for (int i=0; i<2; ++i) {
                        automove_msg.tf_incr[i].translation = change.zero_tl;
                        automove_msg.tf_incr[i].rotation = change.id_ori;
                        automove_msg.del_pos[i] = 0;
                    }

                    // translation
                    if (it->b_tl) {
                        // every step, move 1um delta_pos (from experiment)
                        int incr[3], dif[3];

                        // In the experiment, data1.xd vibrates between two numbers.
                        //   So add this to make sure when 3 values are in range stop this loop.
                        int n_in_range = 0;

                        dif[0] = it->tf.translation.x - act_pos_d[0];
                        dif[1] = it->tf.translation.y - act_pos_d[1];
                        dif[2] = it->tf.translation.z - act_pos_d[2];
                        cout << "difference:"; for (int i=0; i<3; ++i) cout << " " << dif[i]; cout << endl;

                        // approaching the target represented by translation in 3 directions till very close
                        for (int i = 0; i < 3; ++i)
                        {
                            if (abs(dif[i]) < RANGE) {
                                incr[i] = dif[i];
                                n_in_range++;
                            }
                            else {incr[i] = (dif[i] > 0) ? V : -V;}

                            act_pos_d[i] += incr[i];
                        }
                        delta_pos.x = incr[0]; delta_pos.y = incr[1]; delta_pos.z = incr[2];
                        // translation
                        automove_msg.tf_incr[arm_flag].translation = delta_pos; // arm_flag, 0/1 is for left/right arm

                        cout << "n_in_range: " << n_in_range << endl;
                        
                        if (n_in_range == 3) {
                            // at the end of the first segment, label is start
                            if (n_seg == 2) {
                                srv.request.label_from_auto_agent = 0;
                                client.call(srv);
                            };
                            break; // ensure going to next segment.
                        }
                    }
                    
                    // grasper
                    if (it->grasp != 0) {
                        automove_msg.del_pos[arm_flag] = it->grasp;
                    }

                    // rotation
                    if (it->b_rot) {
                        automove_msg.tf_incr[arm_flag].rotation = it->tf.rotation;
                    }

                    automove_msg.hdr.stamp = automove_msg.hdr.stamp.now();
                    
                    pub_motion.publish(automove_msg);
                    ros::spinOnce();
                    loop_rate_pub.sleep();
                }
            }
        }
        ros::Duration(1.0).sleep(); // wait till the arm stops movement.
    }

    return 0;
}
