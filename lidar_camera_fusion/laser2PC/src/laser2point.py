#!/usr/bin/env python3
 
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import ros_numpy
import numpy as np
from std_msgs.msg import Int64MultiArray
import math
 
# P = np.array([235.27027794031173, 0.0, 336.5, -0.0, 0.0, 235.27027794031173, 188.5, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float64).reshape(3, 3)
f = 0.23527027794031173
ox = 0.3365
oy = 0.1885
output = []
angle_min = 0
height = 376
width = 672


 
class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloud", pc2, queue_size = 1)
        self.Kuch = rospy.Publisher("/Kuch", Int64MultiArray, queue_size = 1)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        
    def laserCallback(self, data):
        output = []
        cloud_out = self.laserProj.projectLaser(data)
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_out)
        # for elem in xyz_array:
        #     # elem[0] -= -1.600
        #     # elem[1] -= 0.060
        #     # elem[2] -= 0.600
        #     x = f*elem[0]//elem[2] + ox
        #     y = f*elem[1]//elem[2] + oy
        #     cords = [x, y]
        #     output.append(cords)
        
        # angle_inc = data.angle_increment
        # range_arr = data.ranges
        # output = []
        # x_px = 0
        # y_px = 0
        # for i, dist in enumerate(range_arr):
        #     if dist!=math.inf:
        #         angle = angle_min+i*angle_inc
        #         z = dist*math.sin(angle)
        #         x = dist*math.cos(angle)
        #         y = 1.37
        #         x = f*x//z + ox
        #         y = f*y//z+ oy

        #         if x<0 and abs(x) < 1: x_px=abs(x)*width
        #         elif x>0 and abs(x) < 1: x_px=x*width+width//2
        #         if y<0 and abs(y) < 1: y_px=abs(y)*height
        #         elif y>0 and abs(y)<1: y_px = y*height+height//2
                
        #         output.append([int(x_px), int(y_px)])


        
        rospy.loginfo(output)
        message = Int64MultiArray()
        message.data = output
        self.pcPub.publish(cloud_out)

 
if __name__ == '__main__':
 rospy.init_node("laser2PointCloud")
 l2pc = Laser2PC()
 rospy.spin()