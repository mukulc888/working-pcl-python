import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import  sensor_msgs.point_cloud2 as pc2
import pcl_helper
from vehicle_msgs.msg import TrackCone, Track
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np


pub_arr = rospy.Publisher('/waypoints_arr', Float64MultiArray, queue_size=1)

def callback(data):
    cluster_cords=data.data
    print(cluster_cords)



if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    rospy.Subscriber("/track", Track, callback)
    rospy.spin()
