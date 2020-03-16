pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());


        Mat rgb,depth,depth_copy; 
        try
        {
 
            rgb =   cv::imread("/home/yuan/doc/ppf_ws/src/ppf/dataset/linemod/rgb/0001.png" );   //cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8)->image;//这部分数据应该是用来预测的，后面可以改为读取图片
            depth =  cv::imread("/home/yuan/doc/ppf_ws/src/ppf/dataset/linemod/depth/0001.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH  );     // cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            depth.convertTo(depth, CV_32F);
            depth_copy=depth.clone();
            //hsv.setSrc(rgb);
            //hsv.getMask(final_mask);
           // hsv.getDepthDst(depth);
            //  imshow("rgb",rgb);
            //  imshow("depth",depth);
           // waitKey(0);

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return 1;
        }
        scene->points.clear();
        scene_rgb->points.clear();
        scene_with_normals->points.clear();
        for (int r=0;r<rgb.rows;r++)
        {
            for (int c=0;c<rgb.cols;c++)
            {
                pcl::PointXYZ p;
                pcl::PointXYZRGBA p_rgb;
               


                depth.at<float>(r,c)=double(depth.at<float>(r,c))/1000.0;
                cout << "============..." <<  depth.at<float>(r,c) << endl;

 
                double scene_z = double(depth.at<float>(r,c));
                double scene_x = (c - camera_cx) * scene_z / camera_fx;
                double scene_y = (r - camera_cy) * scene_z / camera_fy;
                p.x = scene_x;
                p.y = scene_y;
                p.z = scene_z;
                scene->points.push_back(p);
              
            }
        }

 
    pcl::visualization::CloudViewer viewerW("ViewerW");
    viewerW.showCloud(scene);
 
    pause();
    return (0);
