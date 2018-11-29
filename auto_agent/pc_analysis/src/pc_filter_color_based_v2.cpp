/** filter the point cloud
 * Check the data in one point and see if this point is under a specified condition.
 * The condition is color based by setting HSV or RPG threshold.
 * Yongming Qin
 * 2018/03
 * v1: The original version. use xbox kinect camera.
 * v2: 2018/08/14 Now we use ZED camera. to continue
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

#include <string>
using namespace std;

float a1, b1, a2, b2, a3, b3; // for testing hsv filter values
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
void color_position(const sensor_msgs::PointCloud2ConstPtr &pc_in);


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_filter_color_based");

    // 169*2 180*2 195 256 168 256
    if (argc == 7) {
        a1 = atof(argv[1]); b1 = atof(argv[2]);
        a2 = atof(argv[3]); b2 = atof(argv[4]);
        a3 = atof(argv[5]); b3 = atof(argv[6]);
    } else if (argc == 1) {
        a1 = 23; b1 = 56; a2 = 200; b2 = 256; a3 = 177; b3 = 256; // yellow
    } else {
        cout << "Wrong arguments!" << endl;
        return -1;
    }
    
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud/cloud_registered", 1, color_position);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("my_pc_filtered", 1);

    // Spin
    /*  while(ros::ok()) {
    ros::spinOnce();
    ros::Duration(1).sleep();
    } */

    while (ros::ok()) {
        ros::spin();
    }
}

/** Callback function of the subscriber
 * Used in the tutorial.
 */
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);

    // Publish the data
    pub.publish(output);
}

/** The callback function for the subscriber written by myself.
 * This is the primary function to filter the input point cloud.
 * And it will publish the filtered point cloud.
 */
void color_position(const sensor_msgs::PointCloud2ConstPtr &pc_in)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*pc_in, *cloud_rgb_in);
    
    //std::vector<int> indices;
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::removeNaNFromPointCloud(*cloud_rgb_ori, *cloud_rgb, indices);
    //ROS_INFO("Size of cloud_rgb: %d", cloud_rgb_ori->points.size());
    /*  for (size_t i = 0; i < 100; ++i)
    std::cout << cloud_rgb->points[i].x << " " << cloud_rgb->points[i].y << " " << cloud_rgb->points[i].z
              << std::endl;
*/
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv_in(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBAtoXYZHSV(*cloud_rgb_in, *cloud_hsv_in);
    cout<< "Size of cloud_hsv_in: " << cloud_hsv_in->points.size() << endl;

    for (size_t i = 0; i < cloud_hsv_in->points.size(); ++i)
    {
        cloud_hsv_in->points[i].x = cloud_rgb_in->points[i].x;
        cloud_hsv_in->points[i].y = cloud_rgb_in->points[i].y;
        cloud_hsv_in->points[i].z = cloud_rgb_in->points[i].z;
    }

    // build the condition
    pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZHSV>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GT, a1*2))); // *2 as different norm
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LT, b1*2)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GT, a2 / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LT, b2 / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GT, a3 / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LT, b3 / 255)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZHSV> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud_hsv_in);
    //condrem.setKeepOrganized(true); // if this exists, nothing is changed.
    // apply filter
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZHSV>);
    condrem.filter(*cloud_color);


    pcl::RadiusOutlierRemoval<pcl::PointXYZHSV> outrem;
    // build the filter
    outrem.setInputCloud(cloud_color);
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius(5);
    // apply filter
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color_radius(new pcl::PointCloud<pcl::PointXYZHSV>);
    outrem.filter(*cloud_color_radius);

    sensor_msgs::PointCloud2 my_pointcloud2;
    pcl::toROSMsg(*cloud_color_radius, my_pointcloud2);
    my_pointcloud2.header.frame_id = "zed_left_camera_frame";
    my_pointcloud2.header.stamp = ros::Time::now();

    pub.publish(my_pointcloud2);
}