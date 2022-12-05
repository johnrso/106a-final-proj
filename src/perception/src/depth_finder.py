#!/usr/bin/env python
# 15hz cam
import sys
import os
import time

import get_xyz

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np
import ros_numpy

class DepthFinderNode(object):
    def __init__(self):
        rospy.init_node("depth_finder", anonymous=True)

        self.pose_pub_path = rospy.get_param("~publishers/pose")
        self.sub_path = rospy.get_param("~subscribers/images")

        self.fx = rospy.get_param("~camera/fx")
        self.fy = rospy.get_param("~camera/fy")
        self.img_h, self.img_w = rospy.get_param("~camera/dims")

        self.ball_dia = rospy.get_param("~segmentation/dia")
        self.radius_min_size = rospy.get_param("~segmentation/radius_min_size")

        self.color_lower = tuple(rospy.get_param("~segmentation/lower_bound"))
        self.color_upper = tuple(rospy.get_param("~segmentation/upper_bound"))

        self.depth_finder =get_xyz.DepthFinder(self.fx, self.fy, self.img_h, self.img_w, self.ball_dia, self.radius_min_size, self.color_upper, self.color_lower)
        self.pose_pub = rospy.Publisher(self.pose_pub_path, PoseStamped, queue_size=10)
        rospy.Subscriber(self.sub_path, Image, self.get_xyz)

        rospy.loginfo("depth_finder: initialized")
        rospy.spin()

    def get_xyz(self, msg):
        rospy.loginfo_once("depth_finder: msg received")
        img_mat = ros_numpy.numpify(msg)
        x, y, z, stat = self.depth_finder.detect_from_color(img_mat, None, use_depth=False)
        if stat:
            pos = PoseStamped()
            pos.pose.position.x = x
            pos.pose.position.y = y
            pos.pose.position.z = z
            self.pose_pub.publish(pos)

            rospy.loginfo(f"depth_finder: ({x}, {y}, {z})")




if __name__ == "__main__":
    try:
        DepthFinderNode()
    except rospy.ROSInterruptException:
        pass
