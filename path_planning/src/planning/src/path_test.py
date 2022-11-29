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
from intera_interface import gripper as robot_gripper

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

try:
    from controller import Controller
except ImportError:
    pass
    
def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")


    Kp = 0.2 * np.array([0.4, 2, 0.5, 0.5, 2, 2, 1])
    Kd = 0.01 * np.array([2, 1, 2, 1.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    custom_controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))

    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.calibrate()
    rospy.sleep(3.0)
    right_gripper.close()
    rospy.sleep(2.0)


    # # 
    # # Add the obstacle to the planning scene here
    # # #
    # pose = PoseStamped()
    # pose.pose.position.x = 0.5
    # pose.pose.position.y = 0.0
    # pose.pose.position.z = 0.0
    # pose.pose.orientation.x = 0.0
    # pose.pose.orientation.y = 0.0
    # pose.pose.orientation.z = 0.0
    # pose.pose.orientation.w = 1.0
    # planner.add_box_obstacle(np.array([[0.4],[1.2],[0.1]]), 'table', pose)

    # # #Create a path constraint for the arm
    # # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.x = -0.676
    orien_const.orientation.y = 0.737;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 10000.0;

    while not rospy.is_shutdown():

        # while not rospy.is_shutdown():
        #     try:
        #         # Initial state
        #         x, y, z = 0.833, -0.074, 0.045
        #         goal_1 = PoseStamped()
        #         goal_1.header.frame_id = "base"

        #         #x, y, and z position
        #         goal_1.pose.position.x = x
        #         goal_1.pose.position.y = y
        #         goal_1.pose.position.z = z

        #         qx, qy, qz, qw = -0.676, 0.737, -0.016, -0.01

        #         #Orientation as a quaternion
        #         goal_1.pose.orientation.x = qx
        #         goal_1.pose.orientation.y = qy
        #         goal_1.pose.orientation.z = qz
        #         goal_1.pose.orientation.w = qw

        #         # Might have to edit this . . . 
        #         plan = planner.plan_to_pose(goal_1, [])
        #         input("Press <Enter> to move the right arm to goal pose 1: ")
        #         if not custom_controller.execute_plan(plan[1]): 
        #             raise Exception("Execution failed")
        #     except Exception as e:
        #         print(e)
        #         traceback.print_exc()
        #     else:
        #         break

        # while not rospy.is_shutdown():
        #     try:
        #         x, y, z = 0.718, -0.155, -0.154

        #         goal_2 = PoseStamped()
        #         goal_2.header.frame_id = "base"

        #         #x, y, and z position
        #         goal_2.pose.position.x = x
        #         goal_2.pose.position.y = y
        #         goal_2.pose.position.z = z

        #         qx, qy, qz, qw = -0.676, 0.737, -0.016, -0.01

        #         #Orientation as a quaternion
        #         goal_2.pose.orientation.x = qx
        #         goal_2.pose.orientation.y = qy
        #         goal_2.pose.orientation.z = qz
        #         goal_2.pose.orientation.w = qw

        #         plan = planner.plan_to_pose(goal_2, [orien_const])
        #         input("Press <Enter> to move the right arm to goal pose 2: ")
        #         if not custom_controller.execute_plan(plan[1]):
        #             raise Exception("Execution failed")
        #         right_gripper.close()
        #     except Exception as e:
        #         print(e)
        #     else:
        #         break

        while not rospy.is_shutdown():
            try:
                x, y, z, = 0.705, 0.075, -0.144
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"

                #x, y, and z position
                goal_3.pose.position.x = x
                goal_3.pose.position.y = y
                goal_3.pose.position.z = z

                qx, qy, qz, qw = -0.676, 0.737, -0.016, -0.01

                #Orientation as a quaternion
                goal_3.pose.orientation.x = qx
                goal_3.pose.orientation.y = qy
                goal_3.pose.orientation.z = qz
                goal_3.pose.orientation.w = qw

                plan = planner.plan_to_pose(goal_3, [])
                input("Press <Enter> to move the right arm to goal pose 3: ")
                if not custom_controller.execute_plan(plan[1]):
                    raise Exception("Execution failed")
                right_gripper.open()
            except Exception as e:
                print(e)
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
