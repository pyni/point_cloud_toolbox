#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
 
#include <pcl/ModelCoefficients.h> 
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> 

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);


  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/outputnew2.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  // draw the point cloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  //viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "registered point cloud");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "registered point cloud");

  // draw the samples
 






 


  // draw the samples
 
 viewer->addPointCloud<pcl::PointXYZ>( cloud, "samples cloudori");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples cloudori");
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples cloudori");
viewer->addCoordinateSystem (0.1);
while (!viewer->wasStopped ())
 {
  viewer->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 }

 



  // draw the samples
  //viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud2, "samples cloud3");
 // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "samples cloud3");
//  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples cloud3");
//viewer->addCoordinateSystem (0.1);
//while (!viewer->wasStopped ())
 // {
 //   viewer->spinOnce (100);
 //   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 // }



}
