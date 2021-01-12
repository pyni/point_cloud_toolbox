# point_cloud_depth_image_transform

## c++ version

transform between point cloud (pcd) and depth image

This is the package compiled in ROS 

readpcd.cpp: This is to read pcd

generatedepthfrompcd.cpp: This is to generate depth image from pcd file

readpcd2rviz.cpp:This is to read pcd to rviz

findmaxplane.cpp:This is to find the max plane for the point cloud with RANSAC algorithm

readpcdfromdepth.cpp:This is to read pcd file from the depth image

pointcloudtostl.cpp:This is to convert ply file (the same with pcd) into stl mesh

automaticllyfindmaxplane.cpp：This is to automatically find the max plane and then turn the point cloud into that plane.

readpcdandfindmaxplaneanddotherotation_and_automally.cpp:This is to automatically find the max plane and change the center of coordination to that plane


usingROSandfindmaxplaneanddotherotation_and_automally.cpp:This is to automatically find the max plane and change the center of coordination to that plane with ROS topic!

extract_subpoints_from_pointclouds.cpp: 从一个整体点云中，提取部分点云（很重要，用循环会很慢）

depthtopointclouds(opencvversion).cpp：This is to generate depth image from pcd file　using opencv c++


此外pcl姿态的一些命令也非常重要，如：

1)各种meshe类型的转换：pcl_png2pcd，pcl_ply2pcd，pcl_ply2obj

2)mesh的采样：pcl_mesh_sampling（pcl_mesh_sampling biaodingkuai.obj output.pcd -leaf_size 0.01）

## python version

第一种：

subscrib_point_cloud_python.py

或者可以参见我之前写的这部分内容：https://github.com/pyni/RGBD_camera_ros_python.git

第二种：

realsense_readingpoindcloudsandsave.py（这个只针对realsense，且只能用python3 且要装pyrealsense，按任意键可以采集）
有些时候depth = geometry.Image(np.asanyarray(depth_frame.get_data()) 要改成下面这个:
depth = geometry.Image(np.asanyarray(depth_frame.get_data()).astype(np.float32)/ 10.0)
![image](https://github.com/pyni/point_cloud_toolbox/blob/master/pic/Screenshot%20from%202020-06-28%2010-40-36.png) 

![image](https://github.com/pyni/point_cloud_toolbox/blob/master/pic/Screenshot%20from%202020-06-28%2010-40-55.png) 

读取RGB并转化为pcd：

read_rgb_depth_to_pcd.py 
(这里需要补充一点就是相机标定方面/camera/depth/color/points和{/camera/color/image_raw，/camera/aligned_depth_to_color/image_raw}还原的深度信息 感觉存在一个偏置，因为后者的这个内参（或者说realsense 各种camera_info的内参）是相对于camera_color_optical_frame的 ，但是/camera/depth/color/points是相对深度相机的link发布的，而/camera/color/image_raw和/camera/aligned_depth_to_color/image_raw是相对于RGB相机的link发布的，所以他们之间相差了一个转换，而read_rgb_depth_to_pcd.py 可以直接将/camera/color/image_raw生成的RGB图和/camera/aligned_depth_to_color/image_raw生成的深度图融合,但需要输入内参 )


## 直接命令形式：

rosrun pcl_ros pointcloud_to_pcd input:=/camera/depth/color/points
