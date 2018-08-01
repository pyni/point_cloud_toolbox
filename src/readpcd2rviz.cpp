// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>






// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "readpcd2rviz");
static tf::TransformBroadcaster br;
    ros::NodeHandle nh;
// Create a ROS publisher for the output point cloud
     ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("npyfiltered_plane", 1);

    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
    pcl::io::loadPCDFile ("/home/yuan/doc/gqcnn_ws/src/gqcnn/examples/ggg.pcd", *cloud2);
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud2, output);
   // output.header.frame_id = std::string("camera_depth_optical_frame");
    output.header.frame_id = std::string("shit_link");
    tf::Quaternion q;
     tf::Transform shit;
    q[0]=-0.5;
    q[1]=0.5;
    q[2]=-0.5;
    q[3]=0.5;
   shit.setRotation(q);
   shit.setOrigin(tf::Vector3(-0.3,0.3,0) );
    //here from this we can see inifitam

    // Publish the data
    while(1)
    {
        br.sendTransform(tf::StampedTransform(shit, ros::Time::now(), "base_link","shit_link"));

    pub.publish (output);
       ros::WallDuration(0.1).sleep();
    }


}
