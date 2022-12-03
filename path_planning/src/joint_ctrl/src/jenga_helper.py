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

def fk_service_client(limb):
    joints = limb.joint_names()
    angles = []

    for j in joints:
        angles.append(limb.joint_angle(j))

    service_name = "ExternalTools/right/PositionKinematicsNode/FKService"
    fk_service_proxy = rospy.ServiceProxy(service_name, SolvePositionFK)
    fk_request = SolvePositionFKRequest()
    joints = JointState()
    joints.name = [f"right_j{i}" for i in range(7)]
    joints.position = angles
    # Add desired pose for forward kinematics
    fk_request.configuration.append(joints)
    # Request forward kinematics from base to "right_gripper_tip" link
    fk_request.tip_names.append('right_gripper_tip')

    try:
        rospy.wait_for_service(service_name, 5.0)
        response = fk_service_proxy(fk_request)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 

    # Check if result valid
    if (response.isValid[0]):
        rospy.loginfo("SUCCESS - Valid Cartesian Solution Found")
        # rospy.loginfo("\nFK Cartesian Solution:\n")
        # rospy.loginfo("------------------")
        # rospy.loginfo("Response Message:\n%s", response)
        # rospy.loginfo("------------------")
        # rospy.loginfo("Original Joint Locations: \n%s", joints)
    else:
        rospy.logerr("INVALID JOINTS - No Cartesian Solution Found.")
        return 

    return response, joints

def ik_service_client(fk_response, joints, is_xy, delta_x, delta_y, delta_z):
    service_name = "ExternalTools/right/PositionKinematicsNode/IKService"
    ik_service_proxy = rospy.ServiceProxy(service_name, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    header = Header(stamp=rospy.Time.now(), frame_id='base')

    # Create a PoseStamped and specify header (specifying a header is very important!)
    pose_stamped = fk_response.pose_stamp[0]
    pose_stamped.header = header
    ik_request.seed_angles = [joints]
    ik_request.use_nullspace_goal = [True]

    if is_xy:
        pose_stamped.pose.position.x += delta_x
        pose_stamped.pose.position.y += delta_y
        null_joints = JointState()
        null_joints.name = [
            "right_j2",
            "right_j4",
            "right_j6"
        ]
        null_joints.position = [joints.position[i] for i in [2, 4, 6]]
        ik_request.nullspace_goal = [null_joints]
        ik_request.nullspace_gain = [0.7]

    else:
        # Set end effector position:
        pose_stamped.pose.position.z += delta_z

        # Set the seed angles:
        null_joints = JointState()
        null_joints.name = [
            "right_j0",
            "right_j2",
            "right_j4",
            "right_j6"
        ]
        null_joints.position = [joints.position[i] for i in [0, 2, 4, 6]]
        ik_request.nullspace_goal = [null_joints]
        ik_request.nullspace_gain = [0.7]

    # Add desired pose for inverse kinematics
    ik_request.pose_stamp.append(pose_stamped)
    # Request inverse kinematics from base to "right_gripper_tip" link
    ik_request.tip_names.append('right_gripper_tip')

    # rospy.loginfo("Running Jenga IK Service Client example.")

    try:
        rospy.wait_for_service(service_name, 5.0)
        response = ik_service_proxy(ik_request)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return

    # Check if result valid, and type of seed ultimately used to get solution
    if (response.result_type[0] > 0):
        #rospy.loginfo("SUCCESS!")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(list(zip(response.joints[0].name, response.joints[0].position)))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        # rospy.loginfo("------------------")
        # rospy.loginfo("Response Message:\n%s", response)
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", response.result_type[0])
        return False

    return limb_joints

def move_z(limb, delta_z):
    rospy.loginfo("Solving FK + IK")
    fk_res, joints = fk_service_client(limb)
    final_joints = ik_service_client(fk_res, joints, False, 0, 0, delta_z)

    rospy.loginfo("Moving to Z Destination")
    limb.move_to_joint_positions(final_joints)
    rospy.sleep(0.5)

def move_xy(limb, delta_x, delta_y):
    rospy.loginfo("Solving FK + IK")
    fk_res, joints = fk_service_client(limb)
    final_joints = ik_service_client(fk_res, joints, True, delta_x, delta_y, 0)

    rospy.loginfo("Moving to XY Destination")
    limb.move_to_joint_positions(final_joints)

    rospy.sleep(0.5)

def rotate_tip(limb, theta_final):
    curr_joints = [(f"right_j{i}", limb.joint_angle(f"right_j{i}")) for i in range(7)]
    curr_joints[-1] = ("right_j6", theta_final)
    final_joints = dict(curr_joints)

    rospy.loginfo("Rotating Tip")
    limb.move_to_joint_positions(final_joints)

    rospy.sleep(1.0)
