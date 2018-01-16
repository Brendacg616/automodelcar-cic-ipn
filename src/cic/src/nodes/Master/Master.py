#!/usr/bin/env python

import rospy
import cv2
from cic.msg import Intersection, \
                    Lane
from master_module import Master, \
                          Task, \
                          LANE_DRIVING

# Global parameters
PWM_STEERING_CENTER = 90
CROSSING_SPEED = -200
VEL_DECREASING_FACTOR = -5
STEERING_CHANGE_FACTOR = -2
MAX_DIST_TO_LINE = 70
MIN_DIST_TO_LINE = 13

# Initiates Master class object
master = Master(PWM_STEERING_CENTER,
                CROSSING_SPEED,
                VEL_DECREASING_FACTOR,
                STEERING_CHANGE_FACTOR,
                MAX_DIST_TO_LINE,
                MIN_DIST_TO_LINE,
                Task(LANE_DRIVING))

def on_new_intersection_msg(msg):

    dist_to_line = msg.distance
    line_angle = msg.angle

    master.dist_to_line = dist_to_line
    master.line_angle = line_angle

    # Task assigner
    master.task_assigner()

    # Task solver
    master.task_solver()
    
    print('Distance to inter. line: '+ str(msg.distance))
    print('Tasks on pile: ' + str(len(master.task_pile)))
    

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