#! /usr/bin/env python
import argparse

import rospy
import numpy as np

import tf2_ros

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from sensor_msgs.msg import JointState
from intera_core_msgs.srv import SolvePositionFK, SolvePositionFKRequest

from intera_interface import gripper as robot_gripper

from jenga_helper import *

# position = [0.00324, -0.1526, -0.2424, -0.2956, 1.4207, 0.6305, 0.4354, 2.5003, 0.0]
# This is the position of the arm right above the table AR tag

INCH_TO_ROBO_UNIT = 0.0266
INCH_TO_CAM_UNIT = 0.0265 # might be iffy, might be anisotropic

def main():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR") # 0.03975
        return

    print("Initializing node... ")
    rospy.init_node("joint_script")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    print("Done.")

    rospy.loginfo("Moving robot to neutral.")
    side = valid_limbs[0]
    limb = intera_interface.Limb(side)
    limb.move_to_neutral()
    rospy.sleep(3.0)

    limb.set_joint_position_speed(0.11)
    
    x_start, y_start, z_start = 0.449, 0.160, 0.079 # Alice: 0.418, 0.305, -0.165
    
    xpiece_0, ypiece_0, zpiece_0 = 0.647, 0.422, -0.166
    xpiece_finals, ypiece_finals, zpiece_finals = [0.717, 0.717, 0.717], [-0.075, -0.101, -0.126], [-0.168, -0.168, -0.168]

    theta_final = 1.724 #correct theta for a 90 degree rotation of the wrist

    sf = 1 #00000
    #sf = INCH_TO_ROBO_UNIT/INCH_TO_CAM_UNIT

      # without inverse
    # position: 
    #     x: 0.4496182396603934
    #     y: 0.15749519781565421
    #     z: 0.07903794581546336
    #   orientation: 
    #     x: 0.7060166518753945
    #     y: 0.7081909874661295
    #     z: -7.334899006126598e-05
    #     w: 0.002450952127636918


    # with inverse
    #       position: 
    #     x: 0.44953039083319135
    #     y: 0.15752531749824905
    #     z: 0.07905270135850939
    #   orientation: 
    #     x: 0.7061524574808812
    #     y: 0.7080561721395316
    #     z: -0.000179366960962833
    #     w: 0.002265329185751231

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Warmup the right gripper
    right_gripper.close()
    rospy.sleep(0.2)
    right_gripper.open()
    rospy.sleep(0.2)

    # gripper_tip [0.450, 0.158, 0.079]. ar_tag [-0.32, -2.06, 6.39]
    start_nums = [12] #[16, 17, 11]
    for i, n in enumerate(start_nums):

        #block_transform = tfBuffer.lookup_transform("ar_marker_15", "usb_cam", rospy.Time())
        block_transform = tfBuffer.lookup_transform("ar_marker_8", f"ar_marker_{n}", rospy.Time())
        final_transform = tfBuffer.lookup_transform("ar_marker_8", "ar_marker_15", rospy.Time())
        b = block_transform.transform.translation
        print("Translation", b)
        xpiece_0, ypiece_0, zpiece_0 = b.z + 0.06, -b.x - 0.003, -0.170 #+ (len(start_nums) - i - 1) * 0.014#-b.z
        print("The expected position is", xpiece_0, ypiece_0, zpiece_0)
        # assert(zpiece_0 > -0.171)         # [0.458, 0.163, -0.132]

        move_xy(limb, xpiece_0 - x_start, ypiece_0 - y_start)
        move_z(limb, zpiece_0 - z_start)
        right_gripper.close()
        rospy.sleep(1.0)
        move_z(limb, z_start - zpiece_0)


        f = final_transform.transform.translation
        xpiece_final, ypiece_final, zpiece_final = f.z + 0.06, -f.x + 0.003, -0.170 + (i + 1) * 0.014
        print("The expected position is", xpiece_final, ypiece_final, zpiece_final)
        move_xy(limb, xpiece_final - xpiece_0, ypiece_final - ypiece_0)
        move_z(limb, zpiece_final - z_start)
        right_gripper.open()
        rospy.sleep(1.0)
        move_z(limb, z_start - zpiece_final)

# -0.148 # -0.162

# Block height = 0.014

        limb.move_to_neutral()
        limb.set_joint_position_speed(0.11)
        rospy.sleep(1.0)



if __name__ == '__main__':
    main()

# TESTING INFO FOR ROBOT FRAME
# x,y,z initial:
# position = [0.767, 0.217, -0.0229]
# oritentation = [0.715, 0.699, 0.030, -0.0131]

# x,y,z with y moved one inch to the left (from POV of guy on computer)
# position = [0.763, 0.199, -0.0226]
# orientation = [0.704, 0.709, 0.033, 0.00644]
# 1 inch ~= 0.018 robo units

# MORE ROBOT FRAME INFO
# x,y,z inital:
# position = [0.798004, 0.225286, -0.0240819]
# orientation = [0.698325, 0.715702, -0.010467, 0.001861]

#x,y,z with x moved 4 inches in (from POV of computer guy)
# position = [0.691438, 0.213201, -0.021821]
# orientation = [0.699490, 0.714482, 0.000635, 0.015098]
# 1 inch ~= 0.0266 robot units

# Test to find Origin (0,0,0) of Robot Base Frame:
# Robot x: 0.239781
# Measured distance to black metal base of robot: ~13.2cm
# Robot x: 0.2922799
# Measured distance to black metal base of robot: ~18.2cm
# Calculated scale factor (robo unit/cm) = 0.010499 == 0.026669 robo unit/inch

# TESTING INFO FOR CAMERA FRAME
# Camera Readings inital:
# position = [0.5719, 1.026, 8.31]
# orientation = [0.676, 0.680, -0.219, 0.189]

# Camera Readings (moved one inch across in x direction):
# position = [0.775, 1.01, 8.32]
# orientation = [0.692, 0.679, -0.212, 0.0ish]



# Camera Readings (one inch across in both x and y direction):
# position = [0.792537, 1.203, 8.267]
# orientation = [0.708, 0.693, -0.01, 0.0ish]

# seems like 1 inch ~= 0.19 cam units (0.2031 diff in x, 0.177 diff in y)

# Real World Readings (from center of AR tag, when it is one across in x and y from initial):
# position = [~1 inch, ~12 inches,~36 inches]


# Test Solutions with the robot trying to path plan with sf = ROBO_UNIT_TO_INCH/CAM_UNIT_TO_INCH == 0.0266/0.19
# Test 1: 
# Robot expected X,Y,Z (in "inches/robo units"): [~28.49/0.7580, ~(-1.75)/-0.046, ~3.014/0.0802]  
# Measured X,Y,Z (in approximate inches): [~22, ~6.33, ~(-9.5)]
# Relative Difference ([expected-measured]/expected): [0.2285, 4.617, 4.152]

# Test 2:
# Robot expected X,Y,Z (in "inches/robo units"): [~24.788/0.65936, ~(-19.244)/-0.5119, ~(-0.1)/-0.00191567]
# Measured X,Y,Z (in approximate inches): [~20, ~(-6.85), ~(-9.5)]
# Relative Difference ([expected-measured]/expected): [0.19316, 0.644, -94]

# Test 3:
# Robot expected X,Y,Z (in "inches/robo units"): [-16.748646/-0.445514, -1.1195488/-0.02978, 14.617/0.3888]
# Measured gripper location from what I think is 0 [~(-24), ~0, ~19]
# Measured X,Y,Z (in approximate inches): [~25.5, ~0, ~(-9.5)]
# Relative Difference ([expected-measured]/expected): [2.5225, 1, 0.35]
## NOTE: for some reason this test causes the pose measured directly and the pose return by the jenga.py to differ highly, idk why


# Test 4:
# Robot expected X,Y,Z (in "inches/robo units"): [18.01559/0.47921413211415465, -4.75315/-0.12643388555944762, 2.99067/0.07955201107457799]
# Measured X,Y,Z (in approximate inches): [~25, ~0.5, ~(-9.5)]
# Relative Difference ([expected-measured]/expected): [-0.3877, 1.1052, 4.1765]




# IF MEASURING FROM CENTER OF BASE based off of test 4 it's like 19 inches for 0.4792 robo units 
