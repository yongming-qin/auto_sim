/******
 * Yongming Qin
 * 20181109 Generate the debridement path to a csv file for DSN paper.
 * Based on motion_path_publish_v5.cpp
 * v1: 
 *
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <limits>


using namespace std;
using namespace Eigen;





int main(int argc, char **argv)
{
    int arm_flag = 0; // left arm

    /*------------------------------------------------*/
    std::ofstream table; // for recording
    table.open("/home/yq/moveit_traj.csv");
    #define DEG2RAD *3.14159265/180

#ifdef SAMIN
    vector<double> jpos_start = {31.5444812775 DEG2RAD, 90.1361312866 DEG2RAD, 22.9244003296 DEG2RAD, 0, 10.3955860138 DEG2RAD, 19.3089828491 DEG2RAD, -2.37392711639 DEG2RAD, 4.69590997696 DEG2RAD, 29.9757232666 DEG2RAD, 90.325050354 DEG2RAD, 22.9270248413 DEG2RAD, 0, -4.25127267838 DEG2RAD, -3.97164392471 DEG2RAD, 46.3926887512 DEG2RAD, 45.3422470093 DEG2RAD}; // 16

    vector<int> pos_start = {-77507, -23265, 14846, -77477, 26066, 13309}; // 6
    vector<double> ori_start = {-0.975777983665, -0.207313895226, -0.069844275713, 0.00283893872984, 0.307242035866, -0.951627194881, 0.218744635582, -0.928775131702, -0.299211472273, -0.920559287071, 0.22624963522, 0.318404972553, 0.366252332926, 0.216660767794, 0.904940545559, 0.135756596923, 0.949667930603, -0.282313525677}; // 18
#else
    vector<double> jpos_start = {29.9050064087,90.4027328491,22.9450378418,0.0,4.61501455307,2.72895121574,45.7141723633,46.3980178833,29.9512634277,90.373046875,22.9234867096,0.0,-4.27671480179,-3.16669940948,48.0067749023,45.5268821716};
    vector<int> pos_start = {-77594,-24483,13761,-77591,25894,13423};
    vector<double> mpos_start = {4072.9621582,11514.4365234,51608.6015625,0.0,17767.7089844,17767.3515625,-17341.4492188,18150.3144531,4079.23510742,11511.6474609,51562.3984375,0.0,-17749.9785156,-17755.3808594,18142.484375,-17333.6191406};
    vector<double> ori_start = {-0.950746119022,-0.231135845184,0.206538185477,
                                -0.257847458124,0.21991994977,-0.94082403183,
                                0.17203630507,-0.947740197182,-0.268685817719,
                                //
                                -0.927713632584,0.215162232518,0.305045247078,
                                0.350480854511,0.220763549209,0.910179436207,
                                0.128493383527,0.951298415661,-0.280215591192};
    vector<double> grasp_start = {1.57, 1.57};

#endif



    vector<int> pos_tmp(pos_start);
    vector<int> pos_end = {-109004, -2788, -14335, -77577, 25926, 13513};
    vector<int> pos_container = {-126241, -14389, -23611};

    int last_seq = 863937;

// column names
    table << "field.runlevel,";
    table << "field.last_seq,";
    for (int i=0; i<6; ++i) table << "field.pos_d" << i << ",";
    for (int i=0; i<18; ++i) table << "field.ori_d" << i << ",";
    for (int i=0; i<6; ++i) table << "field.pos" << i << ",";
    for (int i=0; i<16; ++i) table << "field.jpos" << i << ",";
    for (int i=0; i<16; ++i) table << "field.jvel" << i << ",";


    for (int i=0; i<16; ++i) table << "field.mpos_d" << i << ",";
    for (int i=0; i<16; ++i) table << "field.mpos" << i << ",";
    for (int i=0; i<16; ++i) table << "field.mvel" << i << ",";


    for (int i=0; i<2; ++i) table << "field.grasp_d" << i << ",";
    for (int i=0; i<16; ++i) table << "field.encVals" << i << ",";
    for (int i=0; i<16; ++i) table << "field.current_cmd" << i << ",";
    for (int i=0; i<16; ++i) table << "field.jpos_d" << i << ",";
    table << endl;

// first row is the homing postion with values of jpos, jvel, mpos, mvel
    for (int i = 0; i < 100; ++i) {
        table << 3 << "," << last_seq++ <<","; // field.runlevel, field.last_seq
        // field.pos_d
        for (int i=0; i<6; ++i) table << pos_start[i] << ",";
        // field.ori_d
        for (int i=0; i<18; ++i) table << ori_start[i] << ",";
        //
        //for (int i=0; i<6; ++i) table << 0 << ","; // field.pos
        for (int i=0; i<6; ++i) table << pos_start[i] << ",";

        //for (int i=0; i<16; ++i) table << 0 << ","; // field.jpos
        for (int i=0; i<16; ++i) table << jpos_start[i] << ",";
        for (int i=0; i<16; ++i) table << 0 << ","; // field.jvel


        //for (int i=0; i<16; ++i) table << 0 << ","; // field.mpos_d
        for (int i=0; i<16; ++i) table << mpos_start[i] << ",";
        //for (int i=0; i<16; ++i) table << 0 << ","; // field.mpos
        for (int i=0; i<16; ++i) table << mpos_start[i] << ",";
        for (int i=0; i<16; ++i) table << 0 << ","; // field.mvel


        //for (int i=0; i<2; ++i) table << 0 << ","; // field.grasp_d
        for (int i = 0; i<2; ++i) table << grasp_start[i] << ",";
        for (int i=0; i<16; ++i) table << 0 << ","; // field.encVals
        for (int i=0; i<16; ++i) table << 0 << ","; // field.current_cmd
        //for (int i=0; i<16; ++i) table << 0 << ","; // field.jpos_d
        for (int i=0; i<16; ++i) table << jpos_start[i] << ",";

        table << endl;
    }
    





    int rows = 20000;
    vector<double> interval(6);
    for (int i=0; i<6; ++i) interval[i] = (pos_end[i] - pos_start[i]) / rows;

    // every line
    for (int i=0; i<rows; i++) {
        table << 3 << "," << last_seq++ <<","; // field.runlevel, field.last_seq
        for (int i=0; i<6; ++i) { // field.pos_d
            pos_tmp[i] += interval[i];
            table << pos_tmp[i] << ",";
        }
        // field.orid
        for (int i=0; i<18; ++i) table << ori_start[i] << ",";
        //
        for (int i=0; i<6; ++i) table << 0 << ","; // field.pos
        for (int i=0; i<16; ++i) table << 0 << ","; // field.jpos
        for (int i=0; i<16; ++i) table << 0 << ","; // field.jvel


        for (int i=0; i<16; ++i) table << 0 << ","; // field.mpos_d
        for (int i=0; i<16; ++i) table << 0 << ","; // field.mpos
        for (int i=0; i<16; ++i) table << 0 << ","; // field.mvel


        for (int i=0; i<2; ++i) table << 0 << ","; // field.grasp_d
        for (int i = 0; i<2; ++i) table << grasp_start[i] << ",";
        for (int i=0; i<16; ++i) table << 0 << ","; // field.encVals
        for (int i=0; i<16; ++i) table << 0 << ","; // field.current_cmd
        for (int i=0; i<16; ++i) table << 0 << ","; // field.jpos_d
        table << endl;
    }

    return 0;

    
#ifdef REST 

    if (true) // for future
    {
        if (argc == 1) {
            // Approaching the object.
            change.tf.translation.x = obj_pos[0] + 50000;
            change.tf.translation.y = obj_pos[1] + 20000;
            change.tf.translation.z = obj_pos[2] - 10000;
            change.b_tl = true;
            path.push_back(change);
            // move a little down
            change.tf.translation.x -= 10000;
            path.push_back(change);
            // Close the Grasper. raven_tf not change
            change.keep_tl_reset();
            change.grasp = 1; // 1-5
            change.duration = 1;
            path.push_back(change);
            // move a little up
            change.keep_tl_reset();
            change.tf.translation.x += 40000; // 3cm
            change.b_tl = true;
            path.push_back(change);
            // move to the above of the container
            change.reset();
            change.tf.translation.x = -126241;
            change.tf.translation.y = -14389;
            change.tf.translation.z = -23611;
            change.b_tl = true;
            path.push_back(change);
            // open the grasper
            change.keep_tl_reset();
            change.grasp = -1;
            change.duration = 3;
            path.push_back(change);
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

            cout << "The number of segments(8): " << path.size() << endl;
        }
        else if (stoi(argv[1]) == 0) {
            // Approaching the given target
            change.tf.translation.x = -77830;
            change.tf.translation.y = -20349;
            change.tf.translation.z = 13885;
            change.b_tl = true;
            path.push_back(change);
            cout << "The number of segments(1): " << path.size() << endl;
        }
        

        for (auto p : path) { p.info(); cout << endl; }

        string in_s;
        cout << "Start this segment? y?" << endl;
        while (cin >> in_s)
        {
            if (in_s == "y") {break;}
            else if (in_s == "e") {return 0;}
            else {cout << "Please type \"y\" if you want to start ." << endl;}
        }


        const int RANGE = 5;
        const int V = 3;

        int n_seg = 0;
        for (auto it = path.cbegin(); it != path.cend(); ++it)
        {
            if (ros::ok()) {
                cout << "The segment: " << n_seg << endl;
                n_seg++;
                it->info();
                cout << endl;
                ros::Duration(2.0).sleep();

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
                        
                        if (n_in_range == 3) {
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
#endif
}
