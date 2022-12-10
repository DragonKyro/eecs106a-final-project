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
JENGA_WIDTH_INCH = 1 # should be 15/16 but with leeway
JENGA_HEIGHT_INCH = 9/16
FIRST_JENGA_X = 0.409
FIRST_JENGA_Y = -0.237
SECOND_JENGA_X = FIRST_JENGA_X + INCH_TO_ROBO_UNIT
SECOND_JENGA_Y = FIRST_JENGA_Y + INCH_TO_ROBO_UNIT
TABLE_HEIGHT = -0.164

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
    rospy.sleep(1.5)

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
    start_nums = [9,10,11,12,13,14,16,17,8] #[16, 17, 11]
    jenga_count = 0
    for i, n in enumerate(start_nums):
        row = jenga_count % 3
        layer = jenga_count // 3

        NUM_READINGS = 50
        b = np.zeros(3)
        b_or = np.zeros(4)
        block_transform = tfBuffer.lookup_transform("ar_marker_15", f"ar_marker_{n}", rospy.Time())
        btor = block_transform.transform.rotation
        b_or = np.array([btor.x, btor.y, btor.z, btor.w])
        #block_transform = tfBuffer.lookup_transform("ar_marker_15", "usb_cam", rospy.Time())
        for _ in range(NUM_READINGS):
            block_transform = tfBuffer.lookup_transform("ar_marker_15", f"ar_marker_{n}", rospy.Time())
            #final_transform = tfBuffer.lookup_transform("ar_marker_8", "ar_marker_17", rospy.Time())
            bt = block_transform.transform.translation
            b += np.array([bt.x, bt.y, bt.z])
            rospy.sleep(0.01)
        b = b / NUM_READINGS
        # b_or = b_or / NUM_READINGS

        print("Translation", b)
        xpiece_0, ypiece_0, zpiece_0 = b[2] + 0.105 + 0.00665 + (22/16 - 2)*INCH_TO_ROBO_UNIT, -b[0] - 0.0133 + (0) * INCH_TO_ROBO_UNIT, TABLE_HEIGHT #+ (len(start_nums) - i - 1) * 0.014#-b.z
        print("The expected position is", xpiece_0, ypiece_0, zpiece_0)
        # assert(zpiece_0 > -0.171)         # [0.458, 0.163, -0.132]
        arm_t = tfBuffer.lookup_transform("reference/base", f"reference/right_gripper_tip", rospy.Time()).transform.translation
        x_start, y_start, z_start = arm_t.x, arm_t.y, arm_t.z

        # Pick up
        move_xy(limb, xpiece_0 - x_start, ypiece_0 - y_start)
        roll, pitch, yaw = euler_from_quaternion(b_or[0], b_or[1], b_or[2], b_or[3])
        hand = calc_hand_angle(limb.joint_angle("right_j0"), np.pi/2 - pitch, error = 0.25)
        rotate_tip(limb, hand)
        top_joints = move_z(limb, zpiece_0 - z_start)
        right_gripper.close()
        rospy.sleep(1.0)
        move_z(limb, z_start - zpiece_0, custom_seed=top_joints)

        limb.move_to_neutral()
        limb.set_joint_position_speed(0.11)
        rospy.sleep(1.0)

        # test:
        # Translation: [0.494, 0.001, -0.159]
        # Translation: [0.500, 0.018, -0.165]
        # Drop Off

        if layer % 2 == 0:
            move_xy(limb, FIRST_JENGA_X - x_start, FIRST_JENGA_Y - y_start + row*(JENGA_WIDTH_INCH+(1/16))*INCH_TO_ROBO_UNIT)
            hand = calc_hand_angle(limb.joint_angle("right_j0"), np.pi, error = 0.4)
            rotate_tip(limb, hand) 
        else: #this is for even layers, we need to rotate by 90 and stuff
            move_xy(limb, SECOND_JENGA_X - x_start + (1/8)*INCH_TO_ROBO_UNIT - row*(JENGA_WIDTH_INCH+(1/16))*INCH_TO_ROBO_UNIT, SECOND_JENGA_Y - y_start + (2.5/8)*INCH_TO_ROBO_UNIT)
            hand = calc_hand_angle(limb.joint_angle("right_j0"), np.pi/2, error = 0.4)
            rotate_tip(limb, hand) 

        move_z(limb, zpiece_0 - z_start + (layer)*JENGA_HEIGHT_INCH*INCH_TO_ROBO_UNIT + 0.0065)
        

        # get position of gripper at place
        # read current position of block that the gripper placed
        # compare the two positions to see if it's within an acceptable margin
        # if not, return to neutral
        # try to pickup and re-place the block  


        # getting position of the gripper where it placed the block
        arm_t = tfBuffer.lookup_transform("reference/base", f"reference/right_gripper_tip", rospy.Time()).transform.translation
        x_placed, y_placed, z_placed = arm_t.x, arm_t.y, arm_t.z
        #-------------------------
        
        right_gripper.open()
        rospy.sleep(1.0)
        move_z(limb, z_start - zpiece_0 -  layer*JENGA_HEIGHT_INCH*INCH_TO_ROBO_UNIT)
        
        limb.move_to_neutral()
        limb.set_joint_position_speed(0.11)
        rospy.sleep(1.0)

        #checking position of jenga after being placed, try/except in case ar_tag is not visible so we continue building
        try:
            NUM_READINGS = 50
            b = np.zeros(3)
            b_or = np.zeros(4)
            block_transform = tfBuffer.lookup_transform("ar_marker_15", f"ar_marker_{n}", rospy.Time())
            btor = block_transform.transform.rotation
            b_or = np.array([btor.x, btor.y, btor.z, btor.w])
            for _ in range(NUM_READINGS):
                block_transform = tfBuffer.lookup_transform("ar_marker_15", f"ar_marker_{n}", rospy.Time())
                bt = block_transform.transform.translation
                b += np.array([bt.x, bt.y, bt.z])
                rospy.sleep(0.01)
            b = b / NUM_READINGS
            print("Post Pick-Place Translation", b)
            # NOT SURE WHETHER TO USE SHIFTED VALUES FOR COMPARISON OR NOT
            xpiece_0, ypiece_0, zpiece_0 = b[2] + 0.105 + 0.00665 + (22/16 - 2)*INCH_TO_ROBO_UNIT, -b[0] - 0.0133 + (0) * INCH_TO_ROBO_UNIT, TABLE_HEIGHT #+ (len(start_nums) - i - 1) * 0.014#-b.z
            print("The registered position is", xpiece_0, ypiece_0, zpiece_0)
        except Exception as e:
            print(e)
            continue
        
        #if the distance between robot placed point and actual point of placed jenga block is greater than 2 inches, try to pick up again
        if np.linalg.norm(np.array([xpiece_0-x_placed, ypiece_0, y_placed, zpiece_0 - z_placed])) > 2*INCH_TO_ROBO_UNIT:
            
            # INSTEAD OF COPYPASTING CODE TRY TO modify original for loop so that we can just reinsert current marker value
            arm_t = tfBuffer.lookup_transform("reference/base", f"reference/right_gripper_tip", rospy.Time()).transform.translation
            x_start, y_start, z_start = arm_t.x, arm_t.y, arm_t.z

            # Pick up
            move_xy(limb, xpiece_0 - x_start, ypiece_0 - y_start)
            # DOUBLE CHECK THAT THESE ORIENTATIONS ARE PROPERLY UPDATED TO CURRENT BLOCK
            roll, pitch, yaw = euler_from_quaternion(b_or[0], b_or[1], b_or[2], b_or[3])
            hand = calc_hand_angle(limb.joint_angle("right_j0"), np.pi/2 - pitch, error = 0.25)
            rotate_tip(limb, hand)
            top_joints = move_z(limb, zpiece_0 - z_start)
            right_gripper.close()
            rospy.sleep(1.0)
            move_z(limb, z_start - zpiece_0, custom_seed=top_joints)

            limb.move_to_neutral()
            limb.set_joint_position_speed(0.11)
            rospy.sleep(1.0)

            # test:
            # Translation: [0.494, 0.001, -0.159]
            # Translation: [0.500, 0.018, -0.165]
            # Drop Off

            if layer % 2 == 0:
                move_xy(limb, FIRST_JENGA_X - x_start, FIRST_JENGA_Y - y_start + row*(JENGA_WIDTH_INCH+(1/16))*INCH_TO_ROBO_UNIT)
                hand = calc_hand_angle(limb.joint_angle("right_j0"), np.pi, error = 0.4)
                rotate_tip(limb, hand) 
            else: #this is for even layers, we need to rotate by 90 and stuff
                move_xy(limb, SECOND_JENGA_X - x_start + (1/8)*INCH_TO_ROBO_UNIT - row*(JENGA_WIDTH_INCH+(1/16))*INCH_TO_ROBO_UNIT, SECOND_JENGA_Y - y_start + (2.5/8)*INCH_TO_ROBO_UNIT)
                hand = calc_hand_angle(limb.joint_angle("right_j0"), np.pi/2, error = 0.4)
                rotate_tip(limb, hand) 

            move_z(limb, zpiece_0 - z_start + (layer)*JENGA_HEIGHT_INCH*INCH_TO_ROBO_UNIT + 0.0065)    
        
        
        
        
        jenga_count += 1

        # DYNAMIC TOWER PLANNING

        # f = final_transform.transform.translation
        # xpiece_final, ypiece_final, zpiece_final = f.z + 0.06, -f.x + 0.003, -0.170 + (i + 1) * 0.014
        # print("The expected position is", xpiece_final, ypiece_final, zpiece_final)
        # move_xy(limb, xpiece_final - xpiece_0, ypiece_final - ypiece_0)
        # move_z(limb, zpiece_final - z_start)
        # right_gripper.open()
        # rospy.sleep(1.0)
        # move_z(limb, z_start - zpiece_final)

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
