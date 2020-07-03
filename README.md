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

第二种：

realsense_readingpoindcloudsandsave.py（这个只针对realsense，且只能用python3 且要装pyrealsense，按任意键可以采集）
有些时候depth = geometry.Image(np.asanyarray(depth_frame.get_data()) 要改成下面这个:
depth = geometry.Image(np.asanyarray(depth_frame.get_data()).astype(np.float32)/ 10.0)
![image](https://github.com/pyni/point_cloud_toolbox/blob/master/pic/Screenshot%20from%202020-06-28%2010-40-36.png) 

![image](https://github.com/pyni/point_cloud_toolbox/blob/master/pic/Screenshot%20from%202020-06-28%2010-40-55.png) 
