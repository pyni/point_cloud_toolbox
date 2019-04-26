import time
import rospy
from sensor_msgs.msg import Image, PointCloud2 
import cv2
from cv_bridge import CvBridge  
import numpy as np
from message_filters import ApproximateTimeSynchronizer
import message_filters
import sensor_msgs.point_cloud2 as pc2
 


def image_points_callback(msg):
    gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
    
    pointclouds=[]
    for p in gen:
      pointclouds.append([float(p[0]),float(p[1]),float(p[2])])
    np.save('sss.npy',pointclouds)
    print np.array(pointclouds).shape
rospy.init_node('pointnet_detection')

opts=[ops['loss'], ops['pred'], ops['pre_cls'],ops['normalsprediction'],ops['rotationprediction'],ops['pointclouds_pl']]
feeds=feed_dict
 
sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, image_points_callback) 

rospy.spin()
        

