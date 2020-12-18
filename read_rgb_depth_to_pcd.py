import pyrealsense2 as rs
import numpy as np
import cv2
from open3d import *
import open3d as o3d
if __name__=="__main__":
    color_raw = o3d.io.read_image("/home/yuan/doc/objectpose/iros20-6d-pose-tracking/data/YCB_Video_Dataset/data_organized/0048/0rgb.png")
    depth_raw = o3d.io.read_image("/home/yuan/doc/objectpose/iros20-6d-pose-tracking/data/YCB_Video_Dataset/data_organized/0048/0depth.png")

    rgbd = geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, convert_rgb_to_intensity = False);
    pcd =  geometry.PointCloud.create_from_rgbd_image(rgbd, o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))                                                   
    pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

 
    io.write_point_cloud('./pc_color.pcd', pcd)
    visualization.draw_geometries([pcd])