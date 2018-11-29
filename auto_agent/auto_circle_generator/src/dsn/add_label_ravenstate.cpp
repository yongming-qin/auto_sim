/**
 * We want to add the labels of segments in the csv file.
 * I already added a service in the original r2_control node.
 * Thus this node will call this service if the keyboard 'enter' is pressed.
 * At first it's the start segment. If there is a enter, the trajectory goes to next segment.
 * Yongming
 * 2018/11/27
 */

#include <ros/ros.h>
#include "../Raven_PathPlanner.h"
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <limits>

#include <raven_2/SegLabel.h>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_label_ravenstate");

    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*-----------------------Service------------------------------------------------------*/
    ros::ServiceClient client = n.serviceClient<raven_2::SegLabel>("add_seg_label");
    raven_2::SegLabel srv;

    int label = 0;
    string in_s;


    while (ros::ok())
    {
        
        cout << "go to next segment? press 'enter' " << endl;
        while (cin >> in_s && ros::ok())
        {
            cout << "received input." << endl;
            if (in_s == "\'") {
                cout << "call service" << endl;
                // label is sent to r2_control
                label++;
                srv.request.label_from_auto_agent = label;
                cout << "label: " << label << endl;
                client.call(srv);
            } else if (in_s == "e") {
                break;
            }
        }
    }
    ros::spin();
            
    return 0;
}
