/**
 * From the input point cloud, first use the color based condition to find the object.
 * Then get the position of the center of the point cloud.
 * Yongming Qin
 * 2018/08/14
 * v1: based on pcl_filter_color_based_v1.cpp
 * 20180823, should make a new v2, but did not. I add a publisher to publish the object's position in raven space (um).
 */

#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/filters/voxel_grid.h>

#include <pcl/point_types_conversion.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pc_analysis/obj_pos.h>

#include <string>
#include <iostream>
#include <vector>
using namespace std;

const int m2um = 1000000;

vector<double> th; // 6 hsv filter threshold values
vector<double> raven_cent(3,0); // the result. center position of the colored points. m

void color_position(const sensor_msgs::PointCloud2ConstPtr &pc_in);

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial");

    // from color_detection. The frist two values are multiplied by two in "build the condition"
    if (argc == 7) {
        for (int i = 0; i < 6; ++i) th.push_back( stod(argv[i+1]) );
    } else if (argc == 2) {
        int color = stoi(argv[1]);
        if (color == 0) th = {23, 56, 200, 256, 177, 256}; // yellow
        else if (color == 1) th = {117, 145, 127, 211, 83, 144}; // blue
    } else {
        cout << "Wrong arguments!" << endl;
        return -1;
    }


    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud/cloud_registered", 1, color_position);

    // pulisher that publishes the result
    ros::Publisher pub_pos = nh.advertise<pc_analysis::obj_pos>("obj_pos", 1);
    pc_analysis::obj_pos pos;

    // Subscribe for a period so that the value is updated.
    ros::Time t;
    t = t.now();
    ros::Rate loop_rate(10);
    while (ros::ok() && (t.now() - t).toSec() <= 3600)
    {
        for (int i=0; i < 3; ++i) pos.pos[i] = int(raven_cent[i] * m2um);
        for (int i=3; i < 6; ++i) pos.pos[i] = 0;
        pub_pos.publish(pos);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/** The callback function for the subscriber.
 */
void color_position(const sensor_msgs::PointCloud2ConstPtr &pc_in)
{
    // from ros msg to pcl data type
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*pc_in, *cloud_rgb_in);

    //std::vector<int> indices;
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::removeNaNFromPointCloud(*cloud_rgb_ori, *cloud_rgb, indices);

    cout << "\n\nSize of cloud_rgb_in: " << cloud_rgb_in->points.size() << endl;

    // from rgb to hsv
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv_in(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBAtoXYZHSV(*cloud_rgb_in, *cloud_hsv_in);
    cout << "Size of cloud_hsv: " << cloud_hsv_in->points.size() << endl;

    // Tranmit the position. Not sure if this is necessary.
    for (size_t i = 0; i < cloud_hsv_in->points.size(); ++i)
    {
        cloud_hsv_in->points[i].x = cloud_rgb_in->points[i].x;
        cloud_hsv_in->points[i].y = cloud_rgb_in->points[i].y;
        cloud_hsv_in->points[i].z = cloud_rgb_in->points[i].z;
    }

    // build the condition
    pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZHSV>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GT, th[0]*2)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LT, th[1]*2)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GT, th[2] / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LT, th[3] / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GT, th[4] / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LT, th[5] / 255)));
        
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZHSV> cond_rem;
    cond_rem.setInputCloud(cloud_hsv_in);
    cond_rem.setCondition(range_cond);
    //cond_rem.setKeepOrganized(true); // if this exists, nothing is changed.

    // apply filter
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZHSV>);
    cond_rem.filter(*cloud_color);
    cout << "Size of cloud_color: " << cloud_color->points.size() << endl;


    pcl::RadiusOutlierRemoval<pcl::PointXYZHSV> out_rem;
    // build the filter
    out_rem.setInputCloud(cloud_color);
    out_rem.setRadiusSearch(0.02);
    out_rem.setMinNeighborsInRadius(5);
    // apply filter
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color_radius(new pcl::PointCloud<pcl::PointXYZHSV>);
    out_rem.filter(*cloud_color_radius);

    cout << "Size of cloud_color_radius: " << cloud_color_radius->points.size() << endl;

    //centroid obtaining
    double sum[3];
    for (auto it =cloud_color_radius->begin(); it < cloud_color_radius->end(); ++it) {
        sum[0] += it->x; sum[1] += it->y; sum[2] += it->z;
    }
    double cam_cent[3];
    for (int i=0; i<3; ++i) cam_cent[i] = sum[i]/cloud_color_radius->points.size();
    cout << "centeroid (camera frame):"; for (int i=0; i<3; ++i) cout << " " << cam_cent[i]; cout << endl;

    // transfom the position to world (moveit) frame
    // args="-0.27 0.095 0 -1.5708 0.61 0 0_link zed_left_camera_frame 100"
    double world_cent[3];
    const double theta = 1.008; // rad
    const double T_camera[3] = {-0.26, 0.11, 0.0}; // camera center of two lenses
    world_cent[0] = T_camera[0] + cam_cent[1];
    world_cent[1] = T_camera[1] - ( cam_cent[0]*cos(theta) + cam_cent[2]*sin(theta) );
    world_cent[2] = T_camera[2] - ( cam_cent[0]*sin(theta) - cam_cent[2]*cos(theta) );
    cout << "centeroid (world frame):"; for (int i=0; i<3; ++i) cout << " " << world_cent[i]; cout << endl;

    raven_cent[0] = world_cent[2]; raven_cent[1] = world_cent[1]; raven_cent[2] = -0.21 - world_cent[0];
    cout << "centeroid (raven frame):"; for (auto v : raven_cent) cout << " " << v; cout << endl;
    cout << "centeroid (raven frame m to um):"; for (auto v : raven_cent) cout << " " << long(v * m2um); cout << endl;
    
    /*Eigen::Vector4f centroid;
    pcl::compute3DCentroid (cloud_color_radius, centroid);
    cout << "centroid:"; for (int i=0; i<4; ++i) cout << " " << centroid[i]; cout << endl;*/
   
}