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
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from shape_msgs.msg import SolidPrimitive

from path_planner import PathPlanner
from traj import fit_pos, xy_intercept, detect_bounce, sample_from_traj

class PathPlannerNode(object):
    def __init__(self):
        rospy.init_node("path_plan", anonymous=True)
        # test garbage data
        v_0 = np.array([[1, 1, 4]])
        a = np.array([0, 0, -9.8])
        n = 30
        t = np.linspace(0, 1, n)
        t_batch = np.repeat(np.array([t]).T, 3, 1)
        self.buffer = v_0*t_batch+(a/2)*(t_batch**2) + np.random.normal(scale=0.02, size=(n,3))
        # test garbage data

        self.buffer = []
        self.a_xy = 0
        self.a_z = -9.8
        self.z_fixed = 0.0
        self.x_thresh = 3.0

        self.planning_group = rospy.get_param("~planning_group", "right_arm")
        self.dt = rospy.get_param("~dt", 1/30)

        # pub_paths = rospy.get_param("~publishers")
        sub_paths = rospy.get_param("~subscribers")

        # self.planner = PathPlanner(self.planning_group)
        self.pose_sub = rospy.Subscriber(sub_paths["pose"], PoseStamped, self.add_to_buffer)

        # plotting topic
        self.traj_plot = rospy.Publisher("~traj_plot", PoseArray, 1)
        self.data_plot = rospy.Publisher("~data_plot", PoseArray, 1)
        self.num_plot_samples = 1000

        thread = threading.Thread(target=self.plan, args=())
        thread.daemon = True  # Daemonize thread
        thread.start()

        rospy.loginfo("path_planner: initialized")
        rospy.spin()

    def add_to_buffer(self, msg):
        rospy.loginfo_once("path_planner: received first pos message")

        # extract position from msg and append it to buffer
        pos = msg.pose.position
        self.buffer.append([pos.x, pos.y, pos.z])
        if len(self.buffer) > 50:
            self.buffer.pop(0)
        # TODO: check which frame this is in

    def plan(self):
        rospy.loginfo_once("path_planner: beginning planning server")
        while True:
            # truncate buffer id bounce detected
            # if detect_bounce(self.buffer):
            #     self.buffer = self.buffer[-1:]
            #print(self.buffer)
            if len(self.buffer) < 1:
                rospy.sleep(self.dt)
                continue
            rospy.loginfo_once("path_planner: begin fitting")
            buf_np = np.array(self.buffer)
            n, _ = buf_np.shape

            rospy.loginfo_throttle(1, f"path_planner: num messages --> {n}")

            # fit a parabola to the buffer

            t = np.linspace(0, n*self.dt, n)
            v_fit = fit_pos(t, buf_np)

            # intercepts
            x, y = xy_intercept(v_fit, self.x_thresh)

            # sample 10 posestamped from estimated traj, publish to rviz plot topic
            sample_array = PoseArray()
            sample_array.header.frame_id = "base"
            sample_array.header.stamp = rospy.Time.now()
            for i in range(self.num_plot_samples):
                sample = Pose()
                x_sample, y_sample, z_sample = sample_from_traj(v_fit, self.z_fixed)
                sample.position.x = x_sample
                sample.position.y = y_sample
                sample.position.z = z_sample + buf_np[0, 2]
                sample.orientation.x = 0
                sample.orientation.y = 0
                sample.orientation.z = 0
                sample.orientation.w = 1
                sample_array.poses.append(sample)
            self.traj_plot.publish(sample_array)
            # plot buffer points
            for b in buf_np:
                sample = Pose()
                sample.position.x = b[0]
                sample.position.y = b[1]
                sample.position.z = b[2]
                sample.orientation.x = 0
                sample.orientation.y = 0
                sample.orientation.z = 0
                sample.orientation.w = 1

                sample_array.poses.append(sample)

            rospy.loginfo_once("path_planner: curve fit, beginning to plan")
            # # current_pose = self.planner._group.get_current_pose()

            # goal = PoseStamped()
            # goal.header.frame_id = self.planner.ref_link

            # #x, y, and z position
            # goal.pose.position.x = x
            # goal.pose.position.y = y
            # goal.pose.position.z = current_pose.pose.position.z

            # #Orientation as a quaternion
            # goal.pose.orientation.x = 0.0
            # goal.pose.orientation.y = -1.0
            # goal.pose.orientation.z = 0.0
            # goal.pose.orientation.w = 0.0

            # pcm = PositionConstraint()
            # # pcm.header.frame_id = self.planner.ref_link
            # # pcm.link_name = self.planner.ee_link

            # cbox = SolidPrimitive()
            # cbox.type = SolidPrimitive.BOX
            # cbox.dimensions = [0.1, 0.4, 0.4] # TODO: possibly change
            # pcm.constraint_region.primitives.append(cbox)

            # cbox_pose = Pose()
            # cbox_pose.position.x = current_pose.pose.position.x
            # cbox_pose.position.y = 0.15
            # cbox_pose.position.z = 0.45
            # cbox_pose.orientation.w = 1.0
            # pcm.constraint_region.primitive_poses.append(cbox_pose)

            # plan = self.planner.plan_to_pose(goal, [], [pcm])
            # if not self.planner.execute_plan(plan[1]):
            #     raise Exception("Execution failed")
            # except Exception as e:
            #    print(e)
if __name__ == "__main__":
    try:
        PathPlannerNode()
    except rospy.ROSInterruptException:
        pass
