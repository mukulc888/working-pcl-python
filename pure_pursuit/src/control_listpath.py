#! /usr/bin/env python3
import math
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from vehicle_msgs.msg import WaypointsArray 
from nav_msgs.msg import Odometry
import message_filters
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray

    
def callback(odometry, way_data):
    path = way_data.data
    temp = []
    for i in range(0, len(path)-1, 2):
        temp.append([path[i], path[i+1]])

    path = temp
    point=path[0]
    point_X = point[0]
    point_Y = point[1]
    
    orientation_q = odometry.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
    pitch=0
    roll=0
    
    
    track = Path()
    track.header.frame_id = "odom"
    track.header.stamp = rospy.Time.now()
    #for point_X, point_Y in path:
    #print(yaw) 
    
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    pose = PoseStamped()
    # track.poses.append(PoseStamped((pose.position.x = point_X),
    # (pose.position.y = point_Y),
    # (pose.position.z = 0),
    # (pose.orientation.x = qx),
    # (pose.orientation.y = qy),
    # (pose.orientation.z = qz),
    # (pose.orientation.w = qw)))
    
    pose.pose.position.x = point_X
    pose.pose.position.y = point_Y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    track.poses.append(pose)
    
    pub.publish(track)

    


 
if __name__ == "__main__":
    rospy.init_node("path_for_control", anonymous= True)
    pub = rospy.Publisher('path_segment', Path, queue_size = 1)
    #  sub_way = rospy.Subscriber('waypoints', WaypointsArray, callback)
    
    odom_sub = message_filters.Subscriber('robot_control/odom', Odometry)
    way_sub = message_filters.Subscriber('waypoints_arr', Float64MultiArray)
    
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, way_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    # rate = rospy.Rate(1) # small amount on purpose (20m in 20 sec)
    rospy.spin()