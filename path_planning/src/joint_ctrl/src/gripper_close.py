#!/usr/bin/env python
import rospy

from intera_interface import gripper as robot_gripper

rospy.init_node('gripper_test')

# Set up the right gripper
right_gripper = robot_gripper.Gripper('right_gripper')

# Close the right gripper
print('Closing...')
right_gripper.close()
rospy.sleep(1.0)
 