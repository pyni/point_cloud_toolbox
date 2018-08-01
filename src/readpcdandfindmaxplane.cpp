#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
 
#include <pcl/ModelCoefficients.h> 
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>  
#include <string>
#include <cmath>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/io/png_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointXYZRGBA  PointT;
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


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudcolor (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
 


  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/output.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

 if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/output.pcd", *cloudcolor) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
 if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/output.pcd", *filtered_cloud2) == -1) //* load the file
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
 









  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.002);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  // This is also where we specify the “distance threshold”, which determines how close a point must be to the model in order to be considered an inlier. 
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
// the estimated plane parameters  ax+by+cz+d=0
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
   { //std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                        //       << cloud->points[inliers->indices[i]].y << " "
                                            //   << cloud->points[inliers->indices[i]].z << std::endl;

	filtered_cloud->points.push_back(cloud->points[inliers->indices[i]]);

}


  // draw the samples
 // viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud, "samples cloud2");
 // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "samples cloud2");
 // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0,0.0, "samples cloud2");
 // viewer->addPointCloud<pcl::PointXYZ>( cloud, "samples cloudori");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples cloudori");
 // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples cloudori");
//viewer->addCoordinateSystem (0.1);
//while (!viewer->wasStopped ())
 // {
 //   viewer->spinOnce (100);
  //  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 // }




 
filtered_cloud2->points.clear();

 
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {

  if( 0.0954826*cloud->points[i].x+0.690768*cloud->points[i].y+0.716744*cloud->points[i].z-0.121939  >-0.06
 //and  0.0954826*cloud->points[i].x+0.690768*cloud->points[i].y+0.716744*cloud->points[i].z-0.121939  <-0.005
)
 filtered_cloud2->points.push_back(cloudcolor->points[i]); 

 else
 {
cloudcolor->points[i].x=0;
cloudcolor->points[i].y=0;
cloudcolor->points[i].z=0;
filtered_cloud2->points.push_back(cloudcolor->points[i]); 
 }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
}


std::cout<<filtered_cloud2->points.size()<<std::endl;

    std::string depth_file("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/outputnew.png");
   saveDepthToPNG(depth_file, *filtered_cloud2);


  // draw the samples
  viewer->addPointCloud<pcl::PointXYZRGBA>(filtered_cloud2, "samples cloud3");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "samples cloud3");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples cloud3");
viewer->addCoordinateSystem (0.1);
while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
    // save depth image



}
