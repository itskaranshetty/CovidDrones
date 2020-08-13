#! /usr/bin/python2
# Atharv Sonwane <atharvs.twm@gmail.com>

import math

def dist(p1, p2):
    """
    Returns distance between two any-dimensional points given as tuples
    """
    return math.sqrt(sum([(i - j)**2 for i, j in zip(p1, p2)]))

def unwrap_pose(pose):
    """
    Unwraps geometry_msgs/Pose into two tuples of position and orientation 
    """
    p_x = pose.pose.position.x
    p_y = pose.pose.position.y
    p_z = pose.pose.position.z
    o_x = pose.pose.orientation.x
    o_y = pose.pose.orientation.y
    o_z = pose.pose.orientation.z
    o_w = pose.pose.orientation.w

    return (p_x, p_y, p_z), (o_x, o_y, o_z, o_w)
