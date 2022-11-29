#!/usr/bin/env python
import rospy

from intera_interface import gripper as robot_gripper

rospy.init_node('gripper_test')

# Set up the right gripper
right_gripper = robot_gripper.Gripper('right_gripper')

# Calibrate the gripper (other commands won't work unless you do this first)
print('Calibrating...')
right_gripper.calibrate()
rospy.sleep(2.0)
 
