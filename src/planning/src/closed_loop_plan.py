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
from intera_interface import gripper as robot_gripper

from moveit_msgs.msg import OrientationConstraint, PositionConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray

from path_planner import PathPlanner
from traj import fit_pos, xy_intercept, detect_bounce, sample_from_traj
import std_msgs

try:
    from controller import Controller
except ImportError:
    pass

COLOR_TRANSLUCENT = std_msgs.msg.ColorRGBA(0.0, 0.0, 0.0, 0.5)

class PathPlannerNode(object):
    def __init__(self):
        rospy.init_node("path_plan", anonymous=True)
        # # test garbage data
        # v_0 = np.array([[1, 1, 4]])
        # a = np.array([0, 0, -9.8])
        # n = 30
        # t = np.linspace(0, 1, n)
        # t_batch = np.repeat(np.array([t]).T, 3, 1)
        # self.buffer = v_0*t_batch+(a/2)*(t_batch**2) + np.random.normal(scale=0.02, size=(n,3))
        # test garbage data

        self.buffer = []
        self.a_xy = 0
        self.a_z = -9.8
        self.z_fixed = 0.1
        self.y_fixed = -0.2
        self.x_thresh = 3.0
        self.init_x = 0.4
        self.received = 0

        self.planning_group = rospy.get_param("~planning_group", "right_arm")
        self.dt = rospy.get_param("~dt", 1/30)

        pub_paths = rospy.get_param("~publishers")
        sub_paths = rospy.get_param("~subscribers")

        self.planner = PathPlanner(self.planning_group)
        self.pose_sub = rospy.Subscriber(sub_paths["pose"], PoseStamped, self.add_to_buffer)
        self.constraint_pub = rospy.Publisher(pub_paths["constraints"], MarkerArray, queue_size = 2)

        # plotting topic
        self.traj_plot = rospy.Publisher("~traj_plot", PoseArray, queue_size=1)
        self.data_plot = rospy.Publisher("~data_plot", PoseArray, queue_size=1)
        self.num_plot_samples = 100

        self.right_gripper = robot_gripper.Gripper('right_gripper')
        self.right_gripper.calibrate()

        # try:
        #     init_goal = PoseStamped()
        #     init_goal.header.frame_id = "base"

        #     #x, y, and z position
        #     init_goal.pose.position.x = self.init_x
        #     init_goal.pose.position.y = self.y_fixed
        #     init_goal.pose.position.z = self.z_fixed

        #     #Orientation as a quaternion
        #     init_goal.pose.orientation.x = 0.0
        #     init_goal.pose.orientation.y = -1.0
        #     init_goal.pose.orientation.z = 0.0
        #     init_goal.pose.orientation.w = 0.0

        #     joint_constraint = JointConstraint()
        #     joint_constraint.tolerance_above = 0.1
        #     joint_constraint.tolerance_below = 3.14 / 4
        #     joint_constraint.weight = 1
        #     joint_constraint.joint_name = "torso_t0"

        #     plan = self.planner.plan_to_pose(init_goal, [], [], [joint_constraint])
        #     # input()
        #     if not self.planner.execute_plan(plan[1]):
        #         raise Exception("Execution failed")
        # except Exception as e:
        #     print(e)

        # input('test')
        # import intera_interface
        # from intera_interface import CHECK_VERSION
        # rs = intera_interface.RobotEnable(CHECK_VERSION)
        # init_state = rs.state().enabled
        # rs.enable()

        # limb = Limb("right")
        # start_joints = [-0.685685546875, -0.8380625, -0.5252666015625, 1.8298427734375, 0.5380849609375, 0.7128544921875, 0.23368359375]
        # joint_command = self.make_joint_command(start_joints)
        # limb.set_joint_position_speed(0.3)
        # print(joint_command)
        # limb.set_joint_positions(joint_command)

        self.right_gripper.open()
        rospy.sleep(2.5)
        self.right_gripper.close()
        rospy.sleep(1.0)

        thread = threading.Thread(target=self.plan, args=())
        thread.daemon = True  # Daemonize thread
        thread.start()

        rospy.loginfo("path_planner: initialized")
        rospy.spin()

    def make_joint_command(self, joint_lst):
        names = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
        cmd = dict()
        for i in range(len(names)):
            # limb = Limb("right")
            # current_position = limb.joint_angle(names[i])
            # print(f"tracking curr pos: {names[i]} {current_position}")
            cmd[names[i]] = joint_lst[i]

        return cmd

    def add_to_buffer(self, msg):
        if self.received > 5:
            pos = msg.pose.position
            self.buffer.append([pos.x, pos.y, pos.z])
            rospy.loginfo_once("path_planner: received first pos message")

            # extract position from msg and append it to buffer

            # plot buffer points
            sample_array = PoseArray()
            sample_array.header.frame_id = "base"
            sample_array.header.stamp = rospy.Time.now()
            for b in self.buffer:
                sample = Pose()
                sample.position.x = b[0]
                sample.position.y = b[1]
                sample.position.z = b[2]
                sample.orientation.x = 0
                sample.orientation.y = 0
                sample.orientation.z = 0
                sample.orientation.w = 1

                sample_array.poses.append(sample)
            self.data_plot.publish(sample_array)
            # TODO: check which frame this is in
        self.received += 1

    def plan(self):
        rospy.loginfo_once("path_planner: beginning planning server")
        while True:
            # truncate buffer id bounce detected
            # if detect_bounce(self.buffer):
            #     self.buffer = self.buffer[-1:]
            #print(self.buffer)
            if len(self.buffer) < 20:
                # rospy.sleep(self.dt)
                continue
            rospy.loginfo_once("path_planner: begin fitting")
            buf_np = np.array(self.buffer[-10:])
            n, _ = buf_np.shape

            rospy.loginfo_throttle(1, f"path_planner: num messages --> {n}")

            s1 = time.time()
            # fit a parabola to the buffer

            t = np.linspace(0, n*self.dt, n)
            v_fit = fit_pos(t, buf_np)

            # intercepts
            x, y = xy_intercept(v_fit, self.y_fixed)

            # sample 10 posestamped from estimated traj, publish to rviz plot topic
            sample_array = PoseArray()
            sample_array.header.frame_id = "base"
            sample_array.header.stamp = rospy.Time.now()
            for i in range(self.num_plot_samples):
                sample = Pose()
                x_sample, y_sample, z_sample = sample_from_traj(v_fit, self.z_fixed)
                sample.position.x = x_sample
                sample.position.y = y_sample
                sample.position.z = z_sample
                sample.orientation.x = 0
                sample.orientation.y = 0
                sample.orientation.z = 0
                sample.orientation.w = 1
                sample_array.poses.append(sample)
            self.traj_plot.publish(sample_array)

            e1 = time.time()
            rospy.loginfo(f"total for curve fitting: {e1-s1}")
            rospy.loginfo_once("path_planner: curve fit, beginning to plan")

            s2 = time.time()
            target_goal = PoseStamped()
            target_goal.header.frame_id = "base"

            #x, y, and z position
            target_goal.pose.position.x = x
            target_goal.pose.position.y = self.y_fixed
            target_goal.pose.position.z = self.z_fixed

            #Orientation as a quaternion
            target_goal.pose.orientation.x = 0.0
            target_goal.pose.orientation.y = -1.0
            target_goal.pose.orientation.z = 0.0
            target_goal.pose.orientation.w = 0.0


            pcm = PositionConstraint()
            # pcm.header.frame_id = "base"
            pcm.header.frame_id = self.planner.ref_link
            pcm.link_name = self.planner.ee_link

            cbox = SolidPrimitive()
            cbox.type = SolidPrimitive.BOX
            cbox.dimensions = [1.0, 1.0, 0.1]
            pcm.constraint_region.primitives.append(cbox)

            cbox_pose = Pose()
            cbox_pose.position.x = self.init_x
            cbox_pose.position.y = self.y_fixed
            cbox_pose.position.z = self.z_fixed
            cbox_pose.orientation.w = 1.0
            pcm.constraint_region.primitive_poses.append(cbox_pose)

            joint_constraint = JointConstraint()
            joint_constraint.tolerance_above = 0.1
            joint_constraint.tolerance_below = 3.14 * 3 / 4
            joint_constraint.weight = 1
            joint_constraint.joint_name = "torso_t0"

            orientation_constraint = OrientationConstraint()
            orientation_constraint.link_name = self.planner.ee_link
            orientation_constraint.orientation.x = 0.0
            orientation_constraint.orientation.y = -1.0
            orientation_constraint.orientation.z = 0.0
            orientation_constraint.orientation.w = 0.0
            orientation_constraint.absolute_x_axis_tolerance = 0.5
            orientation_constraint.absolute_y_axis_tolerance = 3.6
            orientation_constraint.absolute_z_axis_tolerance = 0.5

            marker_array_msg = MarkerArray()
            position_constraint_marker = display_box(cbox_pose, cbox.dimensions, pcm.header.frame_id)
            goal_marker = create_marker("goal", target_goal.pose, [0.1,0.1,0.1], [1,0,0], target_goal.header.frame_id, 2)
            marker_array_msg.markers.append(position_constraint_marker)
            marker_array_msg.markers.append(goal_marker)
            self.constraint_pub.publish(marker_array_msg)

            try:
                e2 = time.time()
                rospy.loginfo(f"setting constraints: {e2-s2}")
                plan = self.planner.plan_to_pose(target_goal, [], [pcm], [joint_constraint])
                e3 = time.time()
                rospy.loginfo(f"planning: {e3-e2}")
                if not self.planner.execute_plan(plan[1]):
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)

            self.right_gripper.open()
            input()
            self.buffer = []
            self.received = 0


            self.right_gripper.close()
            rospy.sleep(1.0)


            # rospy.signal_shutdown("done")


def display_box(pose, dimensions, ref_link):
    """Utility function to visualize position constraints."""
    assert len(dimensions) == 3

    # setup cube / box marker type
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "positionconstraint"
    marker.id = 1
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.color = COLOR_TRANSLUCENT
    marker.header.frame_id = ref_link

    # fill in user input
    marker.pose = pose
    marker.scale.x = dimensions[0]
    marker.scale.y = dimensions[1]
    marker.scale.z = dimensions[2]
    return marker

def create_marker(ns, cbox_pose, dimensions, colors, frame_id="reference/right_hand", type=1):
    marker = Marker()

    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = type
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = dimensions[0]
    marker.scale.y = dimensions[1]
    marker.scale.z = dimensions[2]

    # Set the color
    marker.color.r = colors[0]
    marker.color.g = colors[1]
    marker.color.b = colors[2]
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = cbox_pose.position.x
    marker.pose.position.y = cbox_pose.position.y
    marker.pose.position.z = cbox_pose.position.z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.ns = str(ns)
    return marker


if __name__ == "__main__":
    try:
        PathPlannerNode()
    except rospy.ROSInterruptException:
        pass
