#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from asd_exercise.msg import ClosestPoint

# initialize topics published by this node
closestPtPublisher = rospy.Publisher('closest_pt', ClosestPoint, queue_size=2)
closestPtMarkerPublisher = rospy.Publisher('closest_pt_marker', Marker, queue_size=2)


def lidarCallback(scanData):
    """
    The lidar callback function.
    scanData is the lidar scan data, cf. sensor_msgs/LaserScan
    """

    closestPt = ClosestPoint()

    # TODO 1: Add code here for finding the closest point and setting the output message
    rospy.loginfo(rospy.get_caller_id() + ': implement me')

    closestPtPublisher.publish(closestPt)

    
    # TODO 2: Add code below to output the closest point as a marker in rviz
    m = Marker()
    m.header = scanData.header
    m.type = m.CUBE
    # set positions here
    #m.pose.position.x = ..
    #m.pose.position.y = ..
    #m.pose.position.z = ..
    m.pose.orientation.x = 0
    m.pose.orientation.x = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    m.color.r = 1
    m.color.g = 0
    m.color.b = 0
    m.color.a = 1
    m.scale.x = 0.1
    m.scale.y = 0.1
    m.scale.z = 0.1

    closestPtMarkerPublisher.publish(m)


def lidarProcessing():
    """
    The main loop of the lidar processing node
    """

    # Initialize node and subscripe to Lidar scan data
    rospy.init_node('lidar_processing')
    rospy.Subscriber('scan', LaserScan, lidarCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    lidarProcessing()
