#!/usr/bin/env python3

from os import wait
import cv2
import numpy as np
from threading import Thread, enumerate
from queue import Queue
import time
import pandas as pd

# import darknet functions to perform object detections
from darknet import *

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


cap = cv2.VideoCapture('test3.mp4')

def detections(detection_queue, frame_queue):
    while cap.isOpened():
        _, img = cap.read()
        frame_queue.put(img)
        detections, width_ratio, height_ratio = darknet_helper(img, width, height)
        detection_queue.put([detections, width_ratio, height_ratio])
        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()


def drawing(frame_queue, detection_queue):
    i = 87
    wait_frame = 0
    while cap.isOpened():
        cones_cords = []
        detection_list = detection_queue.get()
        img = frame_queue.get()
        img_view = img
        img_h, img_w, chan = img.shape
        detections, width_ratio, height_ratio = detection_list

        for label, confidence, bbox in detections:

            left, top, right, bottom = bbox2points(bbox)
            left, top, right, bottom = int(left * width_ratio), int(top * height_ratio), int(right * width_ratio), int(
                bottom * height_ratio)
            
            if label.lower()=='yellow':
                cv2.rectangle(img_view, (left, top), (right, bottom), (30, 255, 255), 2)
                class_name = 1

            elif label.lower()=='blue':
                cv2.rectangle(img_view, (left, top), (right, bottom), (255, 0, 0), 2)
                class_name = 0

            # code to find out normalized cordinates 

            # x = (left+right)/(2*img_w)
            # y = (top+bottom)/(2*img_h)
            # w = (right-left)/(2*img_w)
            # h = (bottom-top)/(2*img_h)
            # cones_cords.append([int(class_name), x, y, w, h])
        
        # code to create txt file for dataset 

        # data_list = np.array(cones_cords).reshape(-1, 5)
        # cone_class = data_list[:, 0]
        # classes, count = np.unique(cone_class, return_counts=True)
        # wait_frame+=1
        
        # code to anotate

        # if classes.size == 2 and wait_frame>=10:
        #     if count[0]/count[1] > 0.4: 
        #         wait_frame=0
        #         np.set_printoptions(suppress=True)


        #         np.savetxt("new_dataset/img_{}.txt".format(i), data_list, delimiter= " ", fmt='%i %f %f %f %f')
        #         cv2.imwrite("new_dataset/img_{}.jpg".format(i), img)
        #         i+=1
        # elif classes.size > 0 and classes[0] == 0 and wait_frame>=10:
        #     wait_frame=0
        #     np.set_printoptions(suppress=True)
        #     np.savetxt("new_dataset/img_{}.txt".format(i), data_list, delimiter= " ", fmt='%i %f %f %f %f')
        #     cv2.imwrite("new_dataset/img_{}.jpg".format(i), img)
        #     i+=1


        cv2.imshow('image', img_view)
        cv2.waitKey(1)

if __name__ == '__main__':
    detections_queue = Queue()
    frame_queue = Queue(maxsize=1)
    Thread(target=detections, args=(detections_queue, frame_queue)).start()
    Thread(target=drawing, args=(frame_queue, detections_queue)).start()
