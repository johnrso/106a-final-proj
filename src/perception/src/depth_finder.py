#!/usr/bin/env python
# 15hz cam
import sys
import os
import time

import get_xyz

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import numpy as np
import ros_numpy
import message_filters

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

class DepthFinderNode(object):
    def __init__(self):
        rospy.init_node("depth_finder", anonymous=True)

        self.pose_pub_path = rospy.get_param("~publishers/pose")
        self.mask_pub_path = rospy.get_param("~publishers/mask")
        self.rgb_sub_path = rospy.get_param("~subscribers/rgb")
        self.depth_sub_path = rospy.get_param("~subscribers/depth")

        self.fx = rospy.get_param("~camera/fx")
        self.fy = rospy.get_param("~camera/fy")
        self.img_h, self.img_w = rospy.get_param("~camera/dims")

        self.ball_dia = rospy.get_param("~segmentation/dia")
        self.radius_min_size = rospy.get_param("~segmentation/radius_min_size")

        self.color_lower = tuple(rospy.get_param("~segmentation/lower_bound"))
        self.color_upper = tuple(rospy.get_param("~segmentation/upper_bound"))

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_transform = None

        self.depth_finder = get_xyz.DepthFinder(self.fx, self.fy, self.img_h, self.img_w, self.ball_dia, self.radius_min_size, self.color_upper, self.color_lower)
        self.pose_pub = rospy.Publisher(self.pose_pub_path, PoseStamped, queue_size=10)
        self.mask_pub = rospy.Publisher(self.mask_pub_path, Image, queue_size=10)

        rgb_sub = message_filters.Subscriber(self.rgb_sub_path, Image)
        depth_sub = message_filters.Subscriber(self.depth_sub_path, Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub],
                                                          10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.get_xyz)

        rospy.loginfo("depth_finder: initialized")
        rospy.spin()

    def get_xyz(self, rgb_msg, depth_msg):
        rospy.loginfo_once("depth_finder: received sync'd messages")
        rgb_mat = ros_numpy.numpify(rgb_msg)
        depth_mat = ros_numpy.numpify(depth_msg)
        x, y, z, stat, mask = self.depth_finder.detect_from_color(rgb_mat, None, use_depth=False)
        if stat:
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z

            if self.pose_transform is None:
                self.pose_transform = self.tf_buffer.lookup_transform(
                    "base",
                    "rs_camera",
                    rospy.time(0),
                    rospy.Duration(1.0),
                )

            trans_pose_msg = tf2_geometry_msgs.do_transform_pose(pose_msg, self.pose_transform)
            self.pose_pub.publish(trans_pose_msg)

            mask_msg = ros_numpy.msgify(Image, mask, "rgb8")
            self.mask_pub.publish(mask_msg)

            rospy.loginfo(f"depth_finder: {(x, y, z)}")





if __name__ == "__main__":
    try:
        DepthFinderNode()
    except rospy.ROSInterruptException:
        pass
