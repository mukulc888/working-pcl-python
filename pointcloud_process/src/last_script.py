#! /usr/bin/env python3

import rospy
from final_script import Point_CloudProcess

def main():
    rospy.init_node("final" , anonymous= True)
    initiate = Point_CloudProcess()
    rospy.spin()
''' 
    rate = rospy.Rate(1000) # big amount on purpose

    while not rospy.is_shutdown():
        initiate.processing()
        rate.sleep()
'''
    # Spin until ctrl + c
    

if __name__ == '__main__':
    main()