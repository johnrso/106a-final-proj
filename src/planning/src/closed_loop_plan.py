#!/usr/bin/env python
# 15hz cam
import sys
import os
import time
import threading

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import numpy as np
import ros_numpy

from intera_interface import Limb

from moveit_msgs.msg import OrientationConstraint, PositionConstraint
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

from path_planner import PathPlanner
from traj import fit_pos, xy_intercept

class PathPlannerNode(object):
    def __init__(self):
        rospy.init_node("path_plan", anonymous=True)

        self.planner = PathPlanner("right_arm")
        self.buffer = []

        self.dt = rospy.get_param("~dt")
        self.a_xy = 0
        self.a_z = -9.8
        self.dt = 1/30 # camera frequency

        pub_paths = rospy.get_param("~publishers")
        sub_paths = rospy.get_param("~subscribers")

        self.odom_sub = rospy.Subscriber(sub_paths["odom"], PoseStamped, self.add_to_buffer)

        thread = threading.Thread(target=self.plan, args=())
        thread.daemon = True  # Daemonize thread
        thread.start()

        rospy.loginfo("path_planner: initialized")
        rospy.spin()

    def add_to_buffer(self, msg):
        rospy.loginfo_once("path_planner: received first pos message")

        # extract position from msg and append it to buffer
        self.buffer.append(np.array(msg.pose.position))

        # TODO: check which frame this is in

    def plan(self):
        rospy.loginfo("path_planner: beginning planning server")
        while True:
            try:
                buf_np = np.array(self.buffer)
                n, _ = pos_samples.shape
                if buf_np.shape[0] > 1:
                    continue

                # fit a parabola to the buffer

                t = np.linspace(0, n*delta_t, n)
                v_fit = fit_pos(t, pos_samples)

                # intercepts
                x, y = xy_intercept(v_fit)



            except Exception as e:
                print(e)
if __name__ == "__main__":
    try:
        PathPlannerNode()
    except rospy.ROSInterruptException:
        pass