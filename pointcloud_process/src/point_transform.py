#!/usr/bin/env python3
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import  sensor_msgs.point_cloud2 as pc2
import pcl_helper
from vehicle_msgs.msg import TrackCone, Track
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
import numpy as np
import tf 


target_frame = "/zed_frame"
source_frame = "/odom"


def transform_pt(points):
    pass


def callback(data):
    print("Yes")
    list_cords = []
    data = data.data
    for i in range(0, len(data), 2):
        list_cords.append([data[i], data[i+1]])
    transform_pt(list_cords)


if __name__ == '__main__':
    rospy.init_node("FUck_me")
    listener = tf.TransformListener()
    tans_point = tf.TransformerROS()
    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
    print(trans, rot)
    rospy.Subscriber('/waypoints_arr', Float64MultiArray, callback)
    rospy.spin()    