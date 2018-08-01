#include <iostream>
#include <string>
#include <cmath>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/io/png_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGBA PointT;
template <typename T>
void saveDepthToPNG (const std::string &file_name, const pcl::PointCloud<T> &cloud)
{
    pcl::PointCloud<T> depth = cloud;

    double max = 0, min = 0;

    for (int i = 0; i < (int) depth.points.size(); i++) {
        if(depth.points[i].z > max) max = depth.points[i].z;
        if(depth.points[i].z < min) min = depth.points[i].z;
    }

    float scale = (max - min) / 256;

    for (int i = 0; i < (int) depth.points.size(); i++) {
        unsigned int t = depth.points[i].z / scale;

        // make sure pixel value between 0 ~ 255
        t = (t > 255) ? 255 : (t < 0) ? 0 : t;

        depth.points[i].rgba = ((t << 16) | (t << 8) | t);
    }

    pcl::io::savePNGFile(file_name, depth);
}



int main(int argc, char *argv[])
{
    std::string pcd_file("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/output.pcd");

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(pcd_file, *cloud);

    // save origin pcd file to png pictre
    std::string rgb_file("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/output1.png");
    pcl::io::savePNGFile(rgb_file, *cloud);

    // save depth image
    std::string depth_file("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/output2.png");
   saveDepthToPNG(depth_file, *cloud);

    return 0;
}
