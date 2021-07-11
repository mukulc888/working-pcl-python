#!/usr/bin/env python3

import cv2
import sys
import numpy as np
from threading import Thread, enumerate
from queue import Queue
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
# import darknet functions to perform object detections
sys.path.append("/home/mukul888/darknet")
from darknet import *


# load in our YOLOv4 architecture network
network, class_names, class_colors = load_network("/home/mukul888/darknet/cfg/yolov4-tiny-custom.cfg", "/home/mukul888/darknet/data/obj.data", "/home/mukul888/darknet/yolov4-tiny-custom_best.weights")
width = network_width(network)
height = network_height(network)

img_pub = rospy.Publisher('img_corr', Image, queue_size=10)

# darknet helper function to run detection on image
def darknet_helper(img, width, height):
  darknet_image = make_image(width, height, 3)
  img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  img_resized = cv2.resize(img_rgb, (width, height),
                              interpolation=cv2.INTER_LINEAR)
  # get image ratios to convert bounding boxes to proper size
  img_height, img_width, _ = img.shape
  width_ratio = img_width/width
  height_ratio = img_height/height

  # run model on darknet style image to get detections
  copy_image_from_bytes(darknet_image, img_resized.tobytes())
  detections = detect_image(network, class_names, darknet_image)
  free_image(darknet_image)
  return detections, width_ratio, height_ratio

point = []

def callback(data):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, 'bgr8')
    detections(image, data)
 
    


def sub():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/zed/left/image_raw' , Image, callback)
    rospy.spin()



def detections(img, data):
    detections, width_ratio, height_ratio = darknet_helper(img, width, height)
    detection_queue = [detections, width_ratio, height_ratio]
    drawing(img, detection_queue, data)



def drawing(img, detection_queue, data):
    detection_list = detection_queue
    detections, width_ratio, height_ratio = detection_list
    img_bbox = np.zeros(img.shape).astype(np.float32)
    list_detect = []
    img_bbox.fill(255)
    delta = 10

    for label, confidence, bbox in detections:

        left, top, right, bottom = bbox2points(bbox)
        left, top, right, bottom = int(left * width_ratio), int(top * height_ratio), int(right * width_ratio), int(
            bottom * height_ratio)

        if label.lower()=='yellow':
            cv2.rectangle(img, (left-delta, top-delta), (right+delta, bottom+delta), (0, 255, 0), -1)
        elif label.lower()=='blue':
            cv2.rectangle(img, (left-delta, top-delta), (right+delta, bottom+delta), (255, 0, 0), -1)
    bridge = CvBridge()
    msg = Image()
    msg=bridge.cv2_to_imgmsg(img, "bgr8")
    msg.header = data.header
    img_pub.publish(msg)
    cv2.imshow('imgae', img)
    cv2.waitKey(1)

if __name__ == '__main__':
    sub()
    # detections_queue = Queue()
    # frame_queue = Queue(maxsize=1)
    # Thread(target=detections, args=(detections_queue, frame_queue)).start()
    # Thread(target=drawing, args=(frame_queue, detections_queue)).start()

