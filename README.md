# point_cloud_depth_image_transform
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


By pyni_sjtu


python版本的话：
subscrib_point_cloud_python.py



