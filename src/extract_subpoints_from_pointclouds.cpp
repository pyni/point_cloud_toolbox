http://www.pointclouds.org/documentation/tutorials/extract_indices.php




pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>());

pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  seg.setInputCloud (scene_input);
  seg.segment (*inliers, *coefficients);
  // This is also where we specify the “distance threshold”, which determines how close a point must be to the model in order to be considered an inlier. 
  //for (size_t i = 0; i < inliers->indices.size (); ++i)
 //  { 
	//filtered_cloud->points.push_back(scene_input->points[inliers->indices[i]]);

//}

  pcl::ExtractIndices<pcl::PointXYZ> extract;
 
extract.setInputCloud (scene_input_second);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*filtered_cloud);
