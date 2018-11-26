
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h> 
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <fstream> 
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <math.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>      
#define random(x) (rand()%x)


using namespace std; 

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
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonan (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yuan/doc/suction_ws/src/readpcd2rviz/src/3/output.pcd", *cloud) == -1) //* load the file
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
  
double standardlength=0.2;//标准检测长宽
double gridunit=    standardlength/2.0 ;//格子的长宽

  std::vector<double> maskset;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (40);
  sor.setStddevMulThresh (0.4); 
 sor.setKeepOrganized(true);
 sor.filter (*cloud_filtered);

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

  seg.setInputCloud (cloud_filtered);
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



 




//ax+by+cz+d，故法向为（a,b,c）,故问题转化为将(0,0,1）转为（a,b,c）的矩阵


 



double vectorBefore[3]  = {coefficients->values[0],coefficients->values[1],coefficients->values[2]};
double vectorAfter[3] = {0,0, -1};
double rotatinMatrix[4][4];

 




Calculation4d(  vectorBefore , vectorAfter ,   rotatinMatrix );
std::cout << vectorBefore[0]<<"," << vectorBefore[1]<<"," << vectorBefore[2]<<std::endl;
std::cout << rotatinMatrix[0][0]<<"," << rotatinMatrix[0][1]<<"," << rotatinMatrix[0][2]<<"," << rotatinMatrix[0][3]<<std::endl;
std::cout << rotatinMatrix[1][0]<<"," << rotatinMatrix[1][1]<<"," << rotatinMatrix[1][2]<<"," << rotatinMatrix[1][3]<<std::endl;
std::cout << rotatinMatrix[2][0]<<"," << rotatinMatrix[2][1]<<"," << rotatinMatrix[2][2]<<"," << rotatinMatrix[2][3]<<std::endl;
std::cout << rotatinMatrix[3][0]<<"," << rotatinMatrix[3][1]<<"," << rotatinMatrix[3][2]<<"," << rotatinMatrix[3][3]<<std::endl;
//上面是为了采样，即对所有点进行随机采样
//下面是对每个采样点沿着法线进行8点投影







Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
// Here we defined a 45° (PI/4) rotation around the Z axis and a translation on the X axis. 
transform_1 (0,0) = rotatinMatrix[0][0];
transform_1 (0,1) = rotatinMatrix[0][1];
transform_1 (0,2) = rotatinMatrix[0][2];
transform_1 (0,3) =rotatinMatrix[0][3];


transform_1 (1,0) = rotatinMatrix[1][0];
transform_1 (1,1) = rotatinMatrix[1][1];
transform_1 (1,2) = rotatinMatrix[1][2];
transform_1 (1,3) =rotatinMatrix[1][3];


transform_1 (2,0) = rotatinMatrix[2][0];
transform_1 (2,1) = rotatinMatrix[2][1];
transform_1 (2,2) = rotatinMatrix[2][2];
transform_1 (2,3) =rotatinMatrix[2][3];

transform_1 (3,0) = rotatinMatrix[3][0];
transform_1 (3,1) = rotatinMatrix[3][1];
transform_1 (3,2) = rotatinMatrix[3][2];
transform_1 (3,3) =rotatinMatrix[3][3];

//    (row, column)

// Define a translation of 2.5 meters on the x axis.
 
// Print the transformation
printf ("Method #1: using a Matrix4f\n");
std::cout << transform_1 << std::endl;


// Executing the transformation
pcl::PointCloud<pcl::PointXYZ >::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ > ());
/*
void pcl::transformPointCloud(const pcl::PointCloud< PointT > & cloud_in, 
                            pcl::PointCloud< PointT > &  cloud_out,  
                            const Eigen::Matrix4f &  transform  ) 
*/
// Apply an affine transform defined by an Eigen Transform.
pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_1);

cloud_filtered=transformed_cloud;


}
