#! /usr/bin/env python3

import rospy
from RRTstar_PathPlan_Node import RRTstar_PathPlan_Node

def main():
    rospy.init_node('RRTstar_PathPlan_Node')
    RRTstar_PathPlanning = RRTstar_PathPlan_Node()

    rate = rospy.Rate(1000) # big amount on purpose

    while not rospy.is_shutdown():
        RRTstar_PathPlanning.sampleTree()
        rate.sleep()

    # Spin until ctrl + c
    # rospy.spin()

if __name__ == '__main__':
    main()
