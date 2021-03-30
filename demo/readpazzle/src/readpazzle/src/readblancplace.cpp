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
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>              
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/registration/icp.h>
typedef pcl::PointXYZRGBA  PointT;


void CrossProduct(double a[3], double b[3], double ret[3])
{
    ret[0] = a[1] * b[2] - a[2] * b[1];
    ret[1] = a[2] * b[0] - a[0] * b[2];
    ret[2] = a[0] * b[1] - a[1] * b[0];
}

double DotProduct(double a[3], double b[3])
{
    double result;
    result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    return result;
}

double Normalize(double v[3])
{
    double result;

    result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    return result;
}

void Rotate(double** rotateMatrix, double u[3], double ret[3]){
    ret[0]=rotateMatrix[0][0]*u[0]+rotateMatrix[0][1]*u[1]+rotateMatrix[0][2]*u[2];
    ret[1]=rotateMatrix[1][0]*u[0]+rotateMatrix[1][1]*u[1]+rotateMatrix[1][2]*u[2];
    ret[2]=rotateMatrix[2][0]*u[0]+rotateMatrix[2][1]*u[1]+rotateMatrix[2][2]*u[2];
}

void RotationMatrix(double angle, double u[3], double rotatinMatrix[3][3])
{
    double norm = Normalize(u);
    
    u[0] = u[0] / norm;
    u[1] = u[1] / norm;
    u[2] = u[2] / norm;

    rotatinMatrix[0][0] = cos(angle) + u[0] * u[0] * (1 - cos(angle));
    rotatinMatrix[0][1] = u[0] * u[1] * (1 - cos(angle) - u[2] * sin(angle));
    rotatinMatrix[0][2] = u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));

    rotatinMatrix[1][0] = u[2] * sin(angle) + u[0] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][1] = cos(angle) + u[1] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][2] = -u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
      
    rotatinMatrix[2][0] = -u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][1] = u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][2] = cos(angle) + u[2] * u[2] * (1 - cos(angle));

}

//cal transport
void Calculation3d(double vectorBefore[3], double vectorAfter[3], double rotatinMatrix[3][3])
{
    double  rotationAxis[3];
    double rotationAngle;
    CrossProduct(vectorBefore, vectorAfter, rotationAxis);
    rotationAngle = acos(DotProduct(vectorBefore, vectorAfter) / Normalize(vectorBefore) / Normalize(vectorAfter));
    RotationMatrix(rotationAngle, rotationAxis, rotatinMatrix);
}

void Calculation4d(double vectorBefore[3], double vectorAfter[3], double rotatinMatrix[4][4])
{
    double rotate3d[3][3];
	Calculation3d(vectorBefore,vectorAfter, rotate3d);

	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			rotatinMatrix[i][j] = rotate3d[i][j];

	for(int i = 0; i < 3; i++)
	{
		rotatinMatrix[i][3] = 0;
		rotatinMatrix[3][i] = 0;
	}

	rotatinMatrix[3][3] = 1;
}


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr singlepoint (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudcolor (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ >::Ptr filetertemplate(new pcl::PointCloud<pcl::PointXYZ >);




  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yuan/doc/projects_our_lab/wuyuan/readpazzle/1617001138664532.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

 if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("/home/yuan/doc/projects_our_lab/wuyuan/readpazzle/1617001138664532.pcd", *cloudcolor) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
 // if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("/home/yuan/doc/projects_our_lab/wuyuan/readpazzle/1617001138664532.pcd", *filtered_cloud2) == -1) //* load the file
 //  {
 //    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
 //    return (-1);
 //  }

 
 if (pcl::io::loadOBJFile<pcl::PointXYZ > ("/home/yuan/doc/projects_our_lab/wuyuan/readpazzle/square2.obj", *filetertemplate) == -1) //* load the file
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
 


//  viewer->addPointCloud<pcl::PointXYZRGBA>(filtered_cloud3, "samples cloud3");
//  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "samples cloud3");
//  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples cloud3");

// viewer->addCoordinateSystem (0.1);
// while (!viewer->wasStopped ())
//  {
//    viewer->spinOnce (100);
//   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//  }
 






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

   if( coefficients->values[0]*cloud->points[i].x+coefficients->values[1]*cloud->points[i].y+coefficients->values[2]*cloud->points[i].z+coefficients->values[3] >0.005)
 filtered_cloud2->points.push_back(cloudcolor->points[i]); 

//  else
//  {
// cloudcolor->points[i].x=0;
// cloudcolor->points[i].y=0;
// cloudcolor->points[i].z=0;
// filtered_cloud2->points.push_back(cloudcolor->points[i]); 
//  }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
}
 
 



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (filtered_cloud2);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.01);
  sor.filter (*cloud_filtered);
pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered_norgb (new pcl::PointCloud<pcl::PointXYZ >);
pcl::copyPointCloud(*cloud_filtered, *cloud_filtered_norgb);

//pcl::io::savePCDFile( "square.pcd", *filtered_cloud2, true ); 
  
  //pcl::io::saveOBJFile ("/home/yuan/doc/projects_our_lab/wuyuan/readpazzle/square.obj", *cloud_filtered);


    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(*cloud_filtered, mesh.cloud); 
    pcl::io::saveOBJFile("/home/yuan/doc/projects_our_lab/wuyuan/readpazzle/square2.obj", mesh);

 






pcl::IterativeClosestPoint<pcl::PointXYZ , pcl::PointXYZ > icp;
// Set the input source and target
icp.setInputSource (cloud_filtered_norgb);
icp.setInputTarget ( filetertemplate);
 
// Set the max correspondence distance to 5cm (e.g., correspondences with higher
// distances will be ignored)
 //icp.setMaxCorrespondenceDistance (0.005);
// Set the maximum number of iterations (criterion 1)
 icp.setMaximumIterations (10000);
// Set the transformation epsilon (criterion 2)
 icp.setTransformationEpsilon (1e-10);
// Set the euclidean distance difference epsilon (criterion 3)
//icp.setEuclideanFitnessEpsilon (1);
 
// Perform the alignment
icp.align (*cloud_filtered_norgb);
 
// Obtain the transformation that aligned cloud_source to cloud_source_registered
Eigen::Matrix4f transformation = icp.getFinalTransformation ();


    pcl::PointCloud<pcl::PointXYZ >::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ > ());
 

    pcl::transformPointCloud (*filetertemplate, *transformed_cloud, transformation.inverse());


     printf ("Target transformation: using a Matrix4f\n");
      std::cout << transformation << std::endl;






//     // 创建存储点云重心的对象
//     Eigen::Vector4f centroid;
    
//     pcl::compute3DCentroid(*cloud_filtered, centroid);

//     std::cout << "The XYZ coordinates of the centroid are: ("
//               << centroid[0] << ", "
//               << centroid[1] << ", "
//               << centroid[2] << ")." << std::endl;
  
// 	double vectorBefore[3]  = {coefficients->values[0],coefficients->values[1],coefficients->values[2]};
// 	double vectorAfter[3] = {0,0,-1};
// 	double rotatinMatrix[4][4];
// 	Calculation4d(  vectorBefore , vectorAfter ,   rotatinMatrix );
// 	std::cout << vectorBefore[0]<<"," << vectorBefore[1]<<"," << vectorBefore[2]<<std::endl;
// 	std::cout << rotatinMatrix[0][0]<<"," << rotatinMatrix[0][1]<<"," << rotatinMatrix[0][2]<<"," << rotatinMatrix[0][3]<<std::endl;
// 	std::cout << rotatinMatrix[1][0]<<"," << rotatinMatrix[1][1]<<"," << rotatinMatrix[1][2]<<"," << rotatinMatrix[1][3]<<std::endl;
// 	std::cout << rotatinMatrix[2][0]<<"," << rotatinMatrix[2][1]<<"," << rotatinMatrix[2][2]<<"," << rotatinMatrix[2][3]<<std::endl;
// 	std::cout << rotatinMatrix[3][0]<<"," << rotatinMatrix[3][1]<<"," << rotatinMatrix[3][2]<<"," << rotatinMatrix[3][3]<<std::endl; 
	 
//      // pcl::PointXYZ  point;
//      // point.x = centroid[0];
//      // point.y = centroid[1];
//     //  point.z = (-coefficients->values[3]-( coefficients->values[0]*centroid[0]+coefficients->values[1]*centroid[1]))/coefficients->values[2];
// //singlepoint->points.push_back (point);
//    // transform_2 (0,3) = centroid[0];
//   //  transform_2 (1,3) = centroid[1];
//    // transform_2 (2,3) = (-coefficients->values[3]-( coefficients->values[0]*centroid[0]+coefficients->values[1]*centroid[1]))/coefficients->values[2];
 


//     Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//     Eigen::Vector4f vec4d(centroid[0],centroid[1], (-coefficients->values[3]-( coefficients->values[0]*centroid[0]+coefficients->values[1]*centroid[1]))/coefficients->values[2], 1.0);  
//     // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//     // Here we defined a 45° (PI/4) rotation around the Z axis and a translation on the X axis.
//   //  float theta = -M_PI/4; // The angle of rotation in radians
//     transform_1 (0,0) = rotatinMatrix[0][0]; 
//     transform_1 (0,1) = rotatinMatrix[0][1]; 
//     transform_1 (0,2) = rotatinMatrix[0][2]; 
//     transform_1 (0,3) = rotatinMatrix[0][3]; 
//     transform_1 (1,0) = rotatinMatrix[1][0]; 
//     transform_1 (1,1) = rotatinMatrix[1][1]; 
//     transform_1 (1,2) = rotatinMatrix[1][2]; 
//     transform_1 (1,3) = rotatinMatrix[1][3]; 
//     transform_1 (2,0) = rotatinMatrix[2][0]; 
//     transform_1 (2,1) = rotatinMatrix[2][1]; 
//     transform_1 (2,2) = rotatinMatrix[2][2]; 
//     transform_1 (2,3) = rotatinMatrix[2][3]; 
//     transform_1 (3,0) = rotatinMatrix[3][0]; 
//     transform_1 (3,1) = rotatinMatrix[3][1]; 
//     transform_1 (3,2) = rotatinMatrix[3][2]; 
//     transform_1 (3,3) = rotatinMatrix[3][3]; 
//     //    (row, column)

//     // Define a translation of 2.5 meters on the x axis.


//     // Print the transformation
//     printf ("Method #1: using a Matrix4f\n");
//     std::cout << transform_1 << std::endl;

// 	Eigen::Vector4f  newvec4d;
// 	newvec4d=transform_1*vec4d;

//     Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();
//     Eigen::Matrix4f transform_3= Eigen::Matrix4f::Identity();
//     transform_2 (0,3) = -newvec4d[0];
//      transform_2 (1,3) = -newvec4d[1];
//      transform_2 (2,3) = -newvec4d[2];
 


//     transform_3=transform_2*transform_1;
//     // Executing the transformation
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
//     /*
//     void pcl::transformPointCloud(const pcl::PointCloud< PointT > & cloud_in, 
//                                     pcl::PointCloud< PointT > &  cloud_out,  
//                                     const Eigen::Matrix4f &  transform  ) 
//     */
//     // Apply an affine transform defined by an Eigen Transform.

 


//     pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_3);



 

 //std::string depth_file("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/outputnewrotation.png");
 //  saveDepthToPNG(depth_file, *transformed_cloud);
 

  // draw the samples
 viewer->addPointCloud<pcl::PointXYZRGBA>(cloudcolor, "samples cloud2");
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "samples cloud2");
// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0,0.0, "samples cloud2");


 viewer->addPointCloud<pcl::PointXYZ >(transformed_cloud, "samples tml");
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples tml");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0,0.0, "samples tml");



viewer->addCoordinateSystem (0.1);
while (!viewer->wasStopped ())
 {
   viewer->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 }
 
    // save depth image



}
