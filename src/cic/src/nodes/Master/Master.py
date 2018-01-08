#!/usr/bin/env python

import rospy
import cv2
from cic.msg import Intersection, \
                    Lane
import numpy as np

def on_new_intersection_msg(msg):

    return


def main():
    """
    All the node messages will be processed by 
    the master node to set the car's behavior.
    """

    rospy.init_node('Master')
    rospy.loginfo("Master node running...")

    rospy.Subscriber(
        '/crossing_detection', 
        Intersection, 
        on_new_intersection_msg)
    
    rospy.spin()

if __name__ == '__main__':
    main()