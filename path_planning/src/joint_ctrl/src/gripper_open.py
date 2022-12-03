#!/usr/bin/env python
import rospy

from intera_interface import gripper as robot_gripper

rospy.init_node('gripper_test')

# Set up the right gripper
right_gripper = robot_gripper.Gripper('right_gripper')

# Open the right gripper
print('Opening...')
right_gripper.open()
rospy.sleep(1.0)
 
