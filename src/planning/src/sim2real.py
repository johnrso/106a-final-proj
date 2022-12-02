#!/usr/bin/env python
# 15hz cam
import sys
import os
import time

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import numpy as np
import ros_numpy

class Sim2RealNode(object):
    def __init__(self):
        rospy.init_node("sim2real", anonymous=True)

        self.img_pub_path = rospy.get_param("~publishers/img_map")
        self.seg_pub_path = rospy.get_param("~publishers/seg_map")
        self.crop_pub_path = rospy.get_param("~publishers/crop_map")
        self.sub_path = rospy.get_param("~subscribers/images")
        self.img_pub = rospy.Publisher(self.img_pub_path, Image, queue_size=1)
        self.seg_pub = rospy.Publisher(self.seg_pub_path, Image, queue_size=1)
        self.crop_pub = rospy.Publisher(self.crop_pub_path, Image, queue_size=1)

        rospy.Subscriber(self.sub_path, Image, self.sim2real)

        rospy.loginfo("sim2real initialized")
        rospy.spin()

    def sim2real(self, img_msg):
        """
        publishes a simulated image

        input img: 3xHxW RGB image. automatically cropped into 3x256x256.
        output none
        """
        pass

if __name__ == "__main__":
    try:
        Sim2RealNode()
    except rospy.ROSInterruptException:
        pass