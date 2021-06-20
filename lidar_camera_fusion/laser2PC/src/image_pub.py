#!/usr/bin/env python3
 
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
from std_msgs.msg import Int64MultiArray
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def callback(image, output):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image, 'bgr8')
    leng = 0
    data = list(output.data)
    print(data)
    print(len(data))
    for i in range(0,len(data),2):
        x = int(data[i])
        y = int(data[i+1])
        image = cv2.circle(image, (x, y), 4, (0, 0, 255), -1)
    # rospy.loginfo(image.shape)
    cv2.imshow('image', image)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("sensor_fusion")
    image_sub = message_filters.Subscriber('/zed/left/image_raw' , Image)
    info_sub = message_filters.Subscriber("/Kuch", Int64MultiArray)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()