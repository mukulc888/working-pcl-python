#!/usr/bin/env python3

#Noise Removal  X
#?Downsampling  X
#ROI
#Ransac
#Euclidian

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import  sensor_msgs.point_cloud2 as pc2
import pcl_helper
from vehicle_msgs.msg import TrackCone, Track
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import struct
import ctypes
import math


class Point_CloudProcess:
    def __init__(self):
        self.carPosX = 0
        self.carPosY = 0
        #rospy.init_node("final" , anonymous= True)
        
        rospy.Subscriber("/points_fused" , PointCloud2 , self.callback)
        rospy.Subscriber("/robot_control/odom", Odometry, self.odometryCallback)
        self.pub = rospy.Publisher("/velodyne_final" , PointCloud2 , queue_size=1)
        self.pub2 = rospy.Publisher("/track" , Track , queue_size=1)    
        self.pub_arr = rospy.Publisher('/waypoints_arr', Float64MultiArray, queue_size=1)
            
    def odometryCallback(self, odometry):
        self.carPosX = odometry.pose.pose.position.x
        self.carPosY = odometry.pose.pose.position.y    

    #Noise
    def do_statistical_outlier_filtering(self, cloud , mean_k = 10 , thresh = 5.0):
        outlier_filter = cloud.make_statistical_outlier_filter()
        outlier_filter.set_mean_k(mean_k)
        outlier_filter.set_std_dev_mul_thresh(thresh)
        return outlier_filter.filter()

    #Downsampling
    def do_voxel_grid_downsampling(self, pcl_data, leaf_size = 0.1): 
        vox = pcl_data.make_voxel_grid_filter()
        vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
        return vox.filter()

    #ROI
    def do_passthrough(self, pcl_data, filter_axis, axis_min, axis_max):
        passthrough = pcl_data.make_passthrough_filter()
        passthrough.set_filter_field_name(filter_axis)
        passthrough.set_filter_limits(axis_min, axis_max)
        return passthrough.filter()

    #Ransac
    def do_ransac_plane_normal_segmentation(self, point_cloud, input_max_distance):
        segmenter = point_cloud.make_segmenter()

        segmenter.set_model_type(pcl.SACMODEL_PLANE)
        segmenter.set_method_type(pcl.SAC_RANSAC)
        segmenter.set_distance_threshold(input_max_distance)
    
        # segmenter = point_cloud.make_segmenter_normals(ksearch = 50)
        
        # segmenter.set_optimize_coefficients(True)
        
        # segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        
        # segmenter.set_normal_distance_weight(0.1)
        
        # segmenter.set_method_type(pcl.SAC_RANSAC)
        
        # segmenter.set_max_iterations(2000)
        
        # segmenter.set_distance_threshold(input_max_distance)
        
        indices, coefficients = segmenter.segment()

        inliners = point_cloud.extract(indices, negative=False)
        outliers = point_cloud.extract(indices, negative=True)

        return indices, inliners, outliers

    #Euclidian
    def do_euclidian_clustering(self, cloud, cloud_og):
        # Euclidean Clustering

        cloud_xyzrgb = self.store_cloud(cloud_og)
        print(cloud_xyzrgb)
        white_cloud = pcl_helper.XYZRGB_to_XYZ(cloud)
        tree = white_cloud.make_kdtree() 
        ec = white_cloud.make_EuclideanClusterExtraction()

        ec.set_ClusterTolerance(0.3) #0.14, 0.08
        ec.set_MinClusterSize(1)
        ec.set_MaxClusterSize(3500)

        ec.set_SearchMethod(tree)

        cluster_indices = ec.Extract() 

        cluster_color = pcl_helper.random_color_gen()
        #cluster_color = pcl_helper.get_color_list(len(cluster_indices))
        
        #print("No. of cones visible at this moment ", len(cluster_indices))
        
        #print(cluster_indices)
        
        cluster_coords = Track()
        #cone = TrackCone()
        #cluster_coords.cones = []
        color_cluster_point_list = []
        way_points = []
        cord_list=[]
        for j, indices in enumerate(cluster_indices):
            x, y, z = 0, 0, 0
            cone_clusters = []
            for i, indice in enumerate(indices):
                color_cluster_point_list.append([white_cloud[indice][0], white_cloud[indice][1], white_cloud[indice][2], pcl_helper.rgb_to_float(cluster_color)])
                x = x + white_cloud[indice][0]
                y = y + white_cloud[indice][1]
                z = z + white_cloud[indice][2]

            cone_clusters.append(x/len(indices) + self.carPosX)
            cone_clusters.append(y/len(indices) + self.carPosY)
            # print(cone_clusters)
            cord_list.append(cone_clusters)
           
        right_cone = []
        left_cone = []
        for point in cord_list:
            pos = 0
            min_dist = math.inf
            for i, _col_pts in enumerate(cloud_xyzrgb):

                dist = abs(math.dist(point, [_col_pts[0], _col_pts[1]]))
                if dist<min_dist:
                    min_dist = dist
                    pos = i
            r = cloud_xyzrgb[pos][3]
            g = cloud_xyzrgb[pos][4]
            b = cloud_xyzrgb[pos][5]

            if b > g and b > r:
                left_cone.append(point)
                print("left_cone_", r , g, b, point, pos, min_dist)
            elif g>b and g>r:
                right_cone.append(point)
                print("right_cone_", r , g, b, point, pos, min_dist)

        # print("left:", left_cone)
        # print("right", right_cone)




            #print(color_cluster_point_list) 
            #cone.x = x/len(indices)     
            #cone.y = y/len(indices)
            #cone.type = f'cone{j+1}'
            

            #cluster_coords.cones = cone_clusters
            #cluster_coords.data.append(TrackCone(data = cone_clusters))
        cluster_coords.data.append(TrackCone(x = x/len(indices) + self.carPosX,     
                                            y = y/len(indices) + self.carPosY))
            #cluster_coords.cones[j].x = x/len(indices)
            #cluster_coords.cones[j].y = y/len(indices)
            #cluster_coords.cones[j].type = 'cone{k}' 


        

        

        N_CONE = min(len(right_cone), len(left_cone))
        for i in range(N_CONE):
            mean_pt = [float((left_cone[i][0]+right_cone[i][0])/2), float((left_cone[i][1] + right_cone[i][1])/2)] 
            way_points.append(mean_pt)
        


        cluster_cloud = pcl.PointCloud_PointXYZRGB()
        cluster_cloud.from_list(color_cluster_point_list)

        ros_cluster_cloud = pcl_helper.pcl_to_ros(cluster_cloud)

        return cluster_cloud, cluster_coords, way_points


        # tolerance = 0.05
        # min_size = 20
        # max_size = 1500

        # tree = cloud.make_kdtree()
        # extraction_object = cloud.make_EuclideanClusterExtraction()

        # extraction_object.set_ClusterTolerance(tolerance)
        # extraction_object.set_MinClusterSize(min_size)
        # extraction_object.set_MaxClusterSize(max_size)
        # extraction_object.set_SearchMethod(tree)

        # number_of_clusters = len(clusters)
        # colors = random_color_gen()

        # colored_points = []

        # for cluster_id, cluster in enumerate(clusters):
        #     for c, i in enumerate(cluster):
        #         x, y, z = cloud[i][0], cloud[i][1], cloud[i][2]
        #         color = rgb_to_float(colors)
        #         colored_points.append([x, y, z, color])
        
        # return colored_points

        



    def callback(self, input_ros_msg):
        print("callback")
        self.cloud = pcl_helper.ros_to_pcl(input_ros_msg)
    
        #cloud = pcl_helper.XYZRGB_to_XYZ(cloud)

        #cloud_denoised = do_statistical_outlier_filtering(cloud)

        #cloud_downsampled = do_voxel_grid_downsampling(cloud)

        # cloud_roi_x = self.do_passthrough(self.cloud, 'x', 0.0, 30.0)

        # cloud_roi_y = self.do_passthrough(cloud_roi_x, 'y', -5.0, 5.0)

        # _, _, cloud_ransaced = self.do_ransac_plane_normal_segmentation(cloud_roi_y, 0.007)
    
        cloud_clustered, cluster_coords, cone_points = self.do_euclidian_clustering(self.cloud, input_ros_msg)
        self.pub2.publish(cluster_coords)
        msg = Float64MultiArray()
        emplist=[]

        for i, points in enumerate(cone_points):
            emplist.append(points[0])
            emplist.append(points[1])

        
            
        #way_points=np.array(way_points,dtype=np.float64)
        msg.data = emplist
        self.pub_arr.publish(msg) 
        
        cloud_new = pcl_helper.pcl_to_ros(cloud_clustered)
        self.pub.publish(cloud_new)

    def store_cloud(self, pc2_msg):
        cords = []
        gen = pc2.read_points(pc2_msg)
        print(gen)
        for data in gen:            
            test = data[3]
            s = struct.pack('>f', test)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
                    
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            cords.append([data[0], data[1], data[2], r, g, b])
            # print(data)
        return cords

'''
if __name__ == "__main__":
    rospy.init_node("final" , anonymous= True)
    initiate = Point_CloudProcess()
    rate = rospy.Rate(1000) # big amount on purpose

    while not rospy.is_shutdown():
        initiate.processing()
        rate.sleep()
'''
'''
    rospy.Subscriber("/velodyne_points" , PointCloud2 , callback)
    pub = rospy.Publisher("/velodyne_final" , PointCloud2 , queue_size=1)
    pub2 = rospy.Publisher("/track" , Track , queue_size=1)
    rospy.spin()
'''
# end
          