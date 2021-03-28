#!/usr/bin/env python3
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from asd_assignment2.msg import ClosestPoint

# initialize topics published by this node
cmdVelPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=2)


def controllerCallback(closestPt):
    # TODO: Write the callback function for directing the husky towards the closest object point


def controllerLoop():
    """
    The main loop of the hitme node
    """

    # Initialize node and subscripe to Lidar scan data
    rospy.init_node('hit_me')
    rospy.Subscriber('closest_pt', ClosestPoint, controllerCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    controllerLoop()
