#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint, PositionConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
import std_msgs

from path_planner import PathPlanner

try:
    from controller import Controller
except ImportError:
    pass

COLOR_TRANSLUCENT = std_msgs.msg.ColorRGBA(0.0, 0.0, 0.0, 0.5)

def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    marker_pub = rospy.Publisher("/path_test", MarkerArray, queue_size = 2)

    planner = PathPlanner("right_arm")

    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])




    # # 
    # # Add the obstacle to the planning scene here
    # #

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;





    while not rospy.is_shutdown():

        # while not rospy.is_shutdown():
        #     try:
        #         x, y, z = 0.8, 0.05, 0.07
        #         goal_1 = PoseStamped()
        #         goal_1.header.frame_id = "base"

        #         #x, y, and z position
        #         goal_1.pose.position.x = x
        #         goal_1.pose.position.y = y
        #         goal_1.pose.position.z = z

        #         #Orientation as a quaternion
        #         goal_1.pose.orientation.x = 0.0
        #         goal_1.pose.orientation.y = -1.0
        #         goal_1.pose.orientation.z = 0.0
        #         goal_1.pose.orientation.w = 0.0

        #         # Might have to edit this . . . 
        #         plan = planner.plan_to_pose(goal_1, [])
        #         input("Press <Enter> to move the right arm to goal pose 1: ")
        #         if not planner.execute_plan(plan[1]): 
        #             raise Exception("Execution failed")
        #     except Exception as e:
        #         print(e)
        #         traceback.print_exc()
        #     else:
        #         break

        # while not rospy.is_shutdown():
        #     try:
        #         goal_2 = PoseStamped()
        #         goal_2.header.frame_id = "base"

        #         #x, y, and z position
        #         goal_2.pose.position.x = 0.8
        #         goal_2.pose.position.y = 0.3
        #         goal_2.pose.position.z = 0.0

        #         #Orientation as a quaternion
        #         goal_2.pose.orientation.x = 0.0
        #         goal_2.pose.orientation.y = -1.0
        #         goal_2.pose.orientation.z = 0.0
        #         goal_2.pose.orientation.w = 0.0

        #         plan = planner.plan_to_pose(goal_2, [])
        #         input("Press <Enter> to move the right arm to goal pose 2: ")
        #         if not planner.execute_plan(plan[1]):
        #             raise Exception("Execution failed")
        #     except Exception as e:
        #         print(e)
        #     else:
        #         break

        while not rospy.is_shutdown():
            try:
            # if True:
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"


                current_pose = planner._group.get_current_pose()

                #x, y, and z position
                goal_3.pose.position.x = 0.8 # prev 0.6
                goal_3.pose.position.y = -0.2 #-0.2
                goal_3.pose.position.z = 0.0
                # goal_3.pose.position.x = current_pose.pose.position.x
                # goal_3.pose.position.y = current_pose.pose.position.y
                # goal_3.pose.position.z = current_pose.pose.position.z

                #Orientation as a quaternion
                goal_3.pose.orientation.x = 0.0
                goal_3.pose.orientation.y = -1.0
                goal_3.pose.orientation.z = 0.0
                goal_3.pose.orientation.w = 0.0


                pcm = PositionConstraint()
                pcm.header.frame_id = planner.ref_link # planner.ref_link
                # pcm.header.frame_id = "reference/right_gripper_tip" #planner.ref_link
                pcm.link_name = planner.ee_link

                cbox = SolidPrimitive()
                cbox.type = SolidPrimitive.BOX
                # cbox.dimensions = [1,1,1]
                cbox.dimensions = [1.2, 1.2, 0.1]
                # cbox.dimensions = [1.5,1.5,1.5]
                # cbox.dimensions = [0.01, 0.01, 0.01]
                # cbox.dimensions = [0.6, 0.4, 0.6] # TODO: possibly change
                pcm.constraint_region.primitives.append(cbox)
                
                cbox_pose = Pose()
                cbox_pose.position.x = 0.8
                cbox_pose.position.y = 0.3
                cbox_pose.position.z = 0.0
                # cbox_pose.position.y = 0 
                # cbox_pose.position.z = 0
                cbox_pose.orientation.w = 1.0
                # transform = tf_buffer.lookup_transform(planner.ee_link, 
                #                 pcm.header.frame_id, pcm.heade)
                # cbox_pose
                pcm.constraint_region.primitive_poses.append(cbox_pose)

                joint_constraint = JointConstraint() 
                joint_constraint.tolerance_above = 0.1 
                joint_constraint.tolerance_below = 3.14 / 2
                joint_constraint.weight = 1
                joint_constraint.joint_name = "torso_t0"

                orientation_constraint = OrientationConstraint()
                orientation_constraint.link_name = planner.ee_link 
                orientation_constraint.orientation.x = 0.0
                orientation_constraint.orientation.y = -1.0
                orientation_constraint.orientation.z = 0.0
                orientation_constraint.orientation.w = 0.0
                orientation_constraint.absolute_x_axis_tolerance = 10 
                orientation_constraint.absolute_x_axis_tolerance = 10
                orientation_constraint.absolute_x_axis_tolerance = 10


                # display the constraints in rviz
                # self.display_box(cbox_pose, cbox.dimensions)
                marker_array_msg = MarkerArray()
                position_constraint_marker = display_box(cbox_pose, cbox.dimensions, pcm.header.frame_id)
                # position_constraint_marker = create_marker("positionconstraint", cbox_pose, cbox.dimensions, [0,1,0])
                goal_marker = create_marker("goal", goal_3.pose, [0.1,0.1,0.1], [1,0,0], goal_3.header.frame_id, 2)
                marker_array_msg.markers.append(position_constraint_marker)
                marker_array_msg.markers.append(goal_marker)
                # print(marker)
                marker_pub.publish(marker_array_msg)

                # plan = planner.plan_to_pose(goal_3, [orientation_constraint], [pcm], [joint_constraint])
                plan = planner.plan_to_pose(goal_3, [], [pcm], [joint_constraint])
                input("Press <Enter> to move the right arm to goal pose 3: ")
                if not planner.execute_plan(plan[1]):
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
            else:
                break

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


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
