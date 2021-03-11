#! /usr/bin/env python3
import math
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# def Trajectory():
#     path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.0), (13.0, 0.0), (16.0, 1.5), (20.0, 2.5)]

#     pitch=0
#     roll=0
#     carPosX, carPosY = (0.0, 0.0)
#     track = Path()
#     track.header.frame_id = "base_link"
#     track.header.stamp = rospy.Time.now()
#     for point_X, point_Y in path:
#         #yaw = math.atan((point_Y - carPosY)/(point_X - carPosX))
#         #print(yaw)  
#         dir_vec = np.array([point_X - carPosX, point_Y - carPosY])
#         axis = np.array([1.0, 0.0])
 
# #do dot product between them
#         cos_theta = np.dot(dir_vec, axis)/(np.sqrt((point_X - carPosX)**2 + (point_Y - carPosY)**2))
#         yaw = math.acos(_cos_theta)
        
#         qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#         qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#         qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#         qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
#         pose = PoseStamped()
#         # track.poses.append(PoseStamped((pose.position.x = point_X),
#         #                                (pose.position.y = point_Y),
#         #                                (pose.position.z = 0),
#                                     #    (pose.orientation.x = qx),
#                                     #    (pose.orientation.y = qy),
#                                     #    (pose.orientation.z = qz),
#                                     #    (pose.orientation.w = qw)))
                                       
#         pose.pose.position.x = point_X
#         pose.pose.position.y = point_Y
#         pose.pose.position.z = 0.0
#         pose.pose.orientation.x = qx
#         pose.pose.orientation.y = qy
#         pose.pose.orientation.z = qz
#         pose.pose.orientation.w = qw
#         track.poses.append(pose)
        
#         pub.publish(track)

#         carPosX, carPosY = point_X, point_Y

def Path1(i):
    path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.0), (13.0, 0.0), (16.0, 1.5), (20.0, 2.5)]
    
    point_X, point_Y = path[i]
    i += 1
    
    if i==8:
        i = 0

    pitch=0
    roll=0
    carPosX, carPosY = (0.0, 0.0)
    track = Path()
    track.header.frame_id = "map"
    track.header.stamp = rospy.Time.now()
    #for point_X, point_Y in path:
    yaw = math.atan((point_Y - carPosY)/(point_X - carPosX))
    #print(yaw)  
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
    pose = PoseStamped()
        # track.poses.append(PoseStamped((pose.position.x = point_X),
        #                                (pose.position.y = point_Y),
        #                                (pose.position.z = 0),
                                    #    (pose.orientation.x = qx),
                                    #    (pose.orientation.y = qy),
                                    #    (pose.orientation.z = qz),
                                    #    (pose.orientation.w = qw)))
                                       
    pose.pose.position.x = point_X
    pose.pose.position.y = point_Y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    track.poses.append(pose)
        
    pub.publish(track)

    carPosX, carPosY = point_X, point_Y

if __name__ == "__main__":
    rospy.init_node("path_for_control", anonymous= True)
    pub = rospy.Publisher('path_segment', Path, queue_size = 1)
    i = 0
    rate = rospy.Rate(1) # small amount on purpose (20m in 20 sec)

    while not rospy.is_shutdown():
        if(i >= 8):
            i = 0
        Path1(i)
        i+=1
        rate.sleep()
    # rospy.spin()


    
