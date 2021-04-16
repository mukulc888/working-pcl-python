#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu


def callback(data):
    imu_msg = Imu()
    imu_msg = data
    imu_msg.linear_acceleration.x = data.linear_acceleration.x
    imu_msg.linear_acceleration.y = data.linear_acceleration.y*0.01
    imu_msg.linear_acceleration.z = data.linear_acceleration.z*0.01
    imu_pub.publish(imu_msg)


if __name__ == '__main__':
    rospy.init_node("imu_filter")
    rospy.Subscriber('imu', Imu, callback)
    imu_pub = rospy.Publisher('/imu_noisy', Imu, queue_size=10)
    rospy.spin()