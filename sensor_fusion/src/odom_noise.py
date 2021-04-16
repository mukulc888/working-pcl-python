#! /usr/bin/env python3

import rospy 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def callback(data):
    PosX = data.pose.pose.position.x 
    PosY = data.pose.pose.position.y 
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vth= data.twist.twist.angular.z
 
    quaternion = data.pose.pose.orientation
    (r, p, y) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

    y += random.uniform(-0.1, 0.1)
    PosY += random.uniform(-0.1, 0.1)

    odom_quat = quaternion_from_euler(0, 0, y)

    odom = Odometry()
    odom.header.stamp = rospy.Time()
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(PosX, PosY, 0.), Quaternion(*odom_quat))

        # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    odom_pub.publish(odom)



def odom_noise():
    rospy.Subscriber('/odom', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('odom_noise')
    odom_pub = rospy.Publisher('noisy_odom', Odometry, queue_size = 10)
    odom_noise()
