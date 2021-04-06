#!/usr/bin/env python3
import rospy
from math import cos, sin
import tf
import serial
from nav_msgs.msg import Odometry
import schedule
import time
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3




def pub_to_odom():
    l_base = 1
    global wheel_enc
    x = 0
    y = 0
    th = 0
    last_time = rospy.Time.from_sec(0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:      
            arduino = serial.Serial("/dev/ttyUSB0", 9600)
            wheel_enc_dec= float(arduino.readline())
            print(wheel_enc_dec)
        except:
            continue
        curr_time = rospy.Time.now()
        dt = 0.1
        v_left = wheel_enc_dec
        v_right = 0
        del_th = (v_left-v_right)/(2*l_base)*dt
        del_x = ((v_left + v_right)/2)*cos(th)*dt
        del_y = ((v_left+v_right)/2)*sin(th)*dt
        vx = del_x/dt
        vy = del_y/dt
        vth = del_th/dt
        x += del_x
        y += del_y
        th += del_th

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
        odom_broad.sendTransform(
            (x, y, 0.),
            odom_quat,
            curr_time,
            "base_link",
            "odom"
        )
        odom = Odometry()
        odom.header.stamp = curr_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
        last_time = curr_time
        odom_pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('publisher')
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broad = tf.TransformBroadcaster()
    pub_to_odom()
