#!/usr/bin/env python3
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import  sensor_msgs.point_cloud2 as pc2
import pcl_helper
from vehicle_msgs.msg import TrackCone, Track
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2

target_frame = "zed_frame"
source_frame = "lidar_base_link"
bridge = CvBridge()

fx = 235.27027794031173
fy = 235.27027794031173
cx = 336.5
cy = 188.5


def transform_pt(in_point):
    tf2_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf2_buffer)

    my_pose = Pose()
    my_pose.position.x = in_point[0] 
    my_pose.position.y = in_point[1] 
    my_pose.position.z = 0 
    my_pose.orientation.x = 0
    my_pose.orientation.y = 0
    my_pose.orientation.z = 0
    my_pose.orientation.w = 1



    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = my_pose
    pose_stamped.header.frame_id = source_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
    # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf2_buffer.transform(pose_stamped, target_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def cartesian_to_pixel(x, y, z):
    return int((fx*x)//z+cx), int((fy*y)//z + cy)



def callback(img_data, way_pts):
    left_cones = []
    right_cones = []
    mean = []
    emplist = []
    img = bridge.imgmsg_to_cv2(img_data, 'bgr8')
    img = np.array(img, dtype = np.uint8)


    points = way_pts.data
    for i in range(0, len(points), 2):
        curr_pt = [points[i], points[i+1]]
        # if curr_pt[1] > 0 : curr_pt[1]+= 0.175
        # else: curr_pt[0] -= 0.175

        
        point_pose = transform_pt(curr_pt)
        x = point_pose.position.x
        y = point_pose.position.y 
        z = point_pose.position.z/2

        u, v = cartesian_to_pixel(x, y, z)
        print(u, v)
        imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if u in range(0, 672) and v in range(0, 376):
            (h, s, v) = (imgHsv[v, u, 0], imgHsv[v, u, 1], imgHsv[v, u, 2])
            print(h, s, v)
            imgShow = cv2.circle(img, (u, v), 2, (0, 0, 255), -1)

            if h in range(100, 140) and s in range(150, 255) and v in range(0, 255):
                left_cones.append(curr_pt)
            elif h in range(20, 30) and s in range(100, 255) and v in range(100, 255):
                right_cones.append(curr_pt) 
            
    N_CONE = min(len(left_cones), len(right_cones))
    for i in range(N_CONE):
        mean.append((left_cones[i][0]+right_cones[i][0])/2, (left_cones[i][1]+right_cones[i][1])/2)

    for i, points in enumerate(mean):
        emplist.append(points[0])
        emplist.append(points[1])
    

    msg = Float64MultiArray()
    msg.data = emplist
    pub_arr.publish(msg)

    cv2.imshow('image2', imgShow)
    cv2.waitKey(1000)



if __name__ == '__main__':

    rospy.init_node('transform')
    img_sub = message_filters.Subscriber('zed/left/image_raw', Image)
    way_sub = message_filters.Subscriber('waypoints_arr', Float64MultiArray)
    ts = message_filters.ApproximateTimeSynchronizer([img_sub, way_sub], 10, 0.1, allow_headerless=True)
    pub_arr = rospy.Publisher('/points_mean', Float64MultiArray, queue_size=1)
    ts.registerCallback(callback)
    rospy.spin()