/** filter the point cloud
 * Check the data in one point and see if this point is under a specified condition.
 * The condition is color based by setting HSV or RPG threshold.
 * Yongming Qin
 * 2018/03
 * v1: The original version. use xbox kinect camera.
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

float a1, b1, a2, b2, a3, b3; // for testing hsv filter values
ros::Publisher pub;

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

void color_position(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ori (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud_xyz_ori);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_xyz_ori, *cloud_xyz, indices);
  for (size_t i = 0; i < 100; ++i)
    std::cout << cloud_xyz->points[i].data[0] << " " << cloud_xyz->points[i].data[1] << " " << cloud_xyz->points[i].data[2]
              << std::endl;
*/

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb_ori(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*input, *cloud_rgb_ori);
    std::vector<int> indices;
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::removeNaNFromPointCloud(*cloud_rgb_ori, *cloud_rgb, indices);
    //ROS_INFO("Size of cloud_rgb: %d", cloud_rgb_ori->points.size());
    /*  for (size_t i = 0; i < 100; ++i)
    std::cout << cloud_rgb->points[i].x << " " << cloud_rgb->points[i].y << " " << cloud_rgb->points[i].z
              << std::endl;
*/
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBAtoXYZHSV(*cloud_rgb_ori, *cloud_hsv);
    ROS_INFO("***********************************\nSize of cloud_hsv: %d", cloud_hsv->points.size());

    for (size_t i = 0; i < cloud_hsv->points.size(); ++i)
    {
        cloud_hsv->points[i].x = cloud_rgb_ori->points[i].x;
        cloud_hsv->points[i].y = cloud_rgb_ori->points[i].y;
        cloud_hsv->points[i].z = cloud_rgb_ori->points[i].z;
    }
    /*  
  int num = 0;
  for (size_t i = 0; i < cloud_hsv->points.size(); ++i) {
    if (cloud_hsv->points[i].h < 180 && cloud_hsv->points[i].h > 150) num++;
  }
  std::cout << "The number of points of this color is: " << num << std::endl;
*/
    /*    std::cout << cloud_hsv->points[i].h << " "<< cloud_hsv->points[i].s << " "<< cloud_hsv->points[i].v
              << " " << cloud_hsv->points[i].x << " " << cloud_hsv->points[i].y << " " << cloud_hsv->points[i].z
              << std::endl;

*/

    // build the condition
    pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZHSV>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GT, a1)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LT, b1)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GT, a2 / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LT, b2 / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GT, a3 / 255)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LT, b3 / 255)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZHSV> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud_hsv);
    //condrem.setKeepOrganized(true); // if this exists, nothing is changed.
    // apply filter
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZHSV>);
    condrem.filter(*cloud_color);

    ROS_INFO("Size of cloud_color: %d", cloud_color->points.size());
    //ROS_INFO("Width of cloud_color: %d", cloud_color->width);
    //ROS_INFO("Height of cloud_color: %d", cloud_color->height);
    //ROS_INFO("Information of cloud_color: %f", cloud_color->points[4].x);
    //ROS_INFO("Information of cloud_color: %f", cloud_color->points[4].h);
    /*  for (size_t i = 0; i < 100; ++i)
    std::cout << cloud_color->points[i].h << " "<< cloud_color->points[i].s << " "<< cloud_color->points[i].v
              << " " << cloud_color->points[i].x << " " << cloud_color->points[i].y << " " << cloud_color->points[i].z
              << std::endl;
*/

    pcl::RadiusOutlierRemoval<pcl::PointXYZHSV> outrem;
    // build the filter
    outrem.setInputCloud(cloud_color);
    outrem.setRadiusSearch(0.02);
    outrem.setMinNeighborsInRadius(5);
    // apply filter
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_color_radius(new pcl::PointCloud<pcl::PointXYZHSV>);
    outrem.filter(*cloud_color_radius);

    ROS_INFO("Size of cloud_color_radius: %d", cloud_color_radius->points.size());
    //ROS_INFO("Width of cloud_color_radius: %d", cloud_color_radius->width);
    //ROS_INFO("Height of cloud_color_radius: %d", cloud_color_radius->height);
    //ROS_INFO("Information of cloud_color_radius: %f", cloud_color_radius->points[4].h);

    sensor_msgs::PointCloud2 my_pointcloud2;
    pcl::toROSMsg(*cloud_color_radius, my_pointcloud2);
    my_pointcloud2.header.frame_id = "camera_depth_optical_frame";
    my_pointcloud2.header.stamp = ros::Time::now();

    //centroid obtaining
    /*
  pcl::PointXYZHSV min_values, max_values;
  pcl::getMinMax3D(*cloud_color_radius, &min_values, &max_values);
  float x = (min_values.x + max_values.x)/2; float y = (min_values.y + max_values.y)/2; float z = (min_values.z + max_values.z)/2;
*/
    pub.publish(my_pointcloud2);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "my_pcl_tutorial");
    // 169*2 180*2 195 256 168 256
    a1 = atof(argv[1]);
    b1 = atof(argv[2]);
    a2 = atof(argv[3]);
    b2 = atof(argv[4]);
    a3 = atof(argv[5]);
    b3 = atof(argv[6]);
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, color_position);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("my_filtered", 1);

    // Spin
    /*  while(ros::ok()) {
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
*/
    ros::spin();
}