#!/usr/bin/env python3

import cv2
import numpy as np
from threading import Thread, enumerate
from queue import Queue
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
sys.path.append('/home/mukul888/tdr_ws/src/eufs_gazebo/nodes/darknet')
# import darknet functions to perform object detections
from darknet import *


bridge=CvBridge()

# load in our YOLOv4 architecture network
network, class_names, class_colors = load_network("cfg/yolov4-tiny-custom.cfg", "data/obj.data", "yolov4-tiny-custom_best.weights")
width = network_width(network)
height = network_height(network)

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


cap = cv2.VideoCapture('test.mp4')

def image_callback(data):
    image = bridge.imgmsg_to_cv2(data, 'bgr8')
    detections(image)

def Subscriber():
    rospy.init_node('image_listener', anonymous=True)
    sub = rospy.Subscriber('/zed/left/image_raw' , Image, image_callback)
    rospy.spin()



def detections(img):
    detections, width_ratio, height_ratio = darknet_helper(img, width, height)
    detection_queue = [detections, width_ratio, height_ratio]
    drawing(img, detection_queue)



def drawing(img, detection_queue):
    detection_list = detection_queue.get()
    detections, width_ratio, height_ratio = detection_list

    for label, confidence, bbox in detections:

        left, top, right, bottom = bbox2points(bbox)
        left, top, right, bottom = int(left * width_ratio), int(top * height_ratio), int(right * width_ratio), int(
            bottom * height_ratio)

        if label.lower()=='yellow':
            cv2.rectangle(img, (left, top), (right, bottom), (30, 255, 255), 2)
        elif label.lower()=='blue':
            cv2.rectangle(img, (left, top), (right, bottom), (255, 0, 0), 2)


if __name__ == '__main__':s
    Subscriber()
    # detections_queue = Queue()
    # frame_queue = Queue(maxsize=1)
    # Thread(target=detections, args=(detections_queue, frame_queue)).start()
    # Thread(target=drawing, args=(frame_queue, detections_queue)).start()

