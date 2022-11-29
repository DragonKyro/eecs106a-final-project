#! /usr/bin/env python
import argparse

import rospy
import numpy as np

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from sensor_msgs.msg import JointState
from intera_core_msgs.srv import SolvePositionFK, SolvePositionFKRequest

from intera_interface import gripper as robot_gripper


def fk_service_client(limb):
    joints = limb.joint_names()
    angles = []

    for j in joints:
        angles.append(limb.joint_angle(j))

    service_name = "ExternalTools/right/PositionKinematicsNode/FKService"
    fk_service_proxy = rospy.ServiceProxy(service_name, SolvePositionFK)
    fk_request = SolvePositionFKRequest()
    joints = JointState()
    # YOUR CODE HERE
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
        rospy.loginfo("\nFK Cartesian Solution:\n")
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", response)
        rospy.loginfo("------------------")
        rospy.loginfo("Original Joint Locations: \n%s", joints)
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
        ik_request.nullspace_gain = [0.4]

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
        ik_request.nullspace_gain = [0.4]

    # Add desired pose for inverse kinematics
    ik_request.pose_stamp.append(pose_stamped)
    # Request inverse kinematics from base to "right_gripper_tip" link
    ik_request.tip_names.append('right_gripper_tip')

    rospy.loginfo("Running Jenga IK Service Client example.")

    try:
        rospy.wait_for_service(service_name, 5.0)
        response = ik_service_proxy(ik_request)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return

    # Check if result valid, and type of seed ultimately used to get solution
    if (response.result_type[0] > 0):
        rospy.loginfo("SUCCESS!")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(list(zip(response.joints[0].name, response.joints[0].position)))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", response)
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

def move_xy(limb, delta_x, delta_y):
    rospy.loginfo("Solving FK + IK")
    fk_res, joints = fk_service_client(limb)
    final_joints = ik_service_client(fk_res, joints, True, delta_x, delta_y, 0)

    rospy.loginfo("Moving to XY Destination")
    limb.move_to_joint_positions(final_joints)


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return

    print("Initializing node... ")
    rospy.init_node("joint_script")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.loginfo("Enabling robot...")
    rs.enable()
    print("Done.")
    # move(args.limb)

    rospy.loginfo("Moving robot to neutral")
    side = args.limb
    limb = intera_interface.Limb(side)
    limb.move_to_neutral()
    rospy.sleep(3.0)

    limb.set_joint_position_speed(0.15)
    
    # Delta z for neutral to table is 0.229 for alan
    z_start = 0.078
    z_table = -0.164
    delta_z = z_table - z_start

    x_start, y_start = 0.448, 0.159
    xpiece_0, ypiece_0 = 0.843, 0.195
    xpiece_final, ypiece_final = 0.822, 0.002
    delta_x_piece = xpiece_final - xpiece_0
    delta_y_piece = ypiece_final - ypiece_0

    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # WArmup the right gripper
    right_gripper.close()
    rospy.sleep(0.2)
    right_gripper.open()
    rospy.sleep(0.2)

    move_xy(limb, xpiece_0 - x_start, ypiece_0 - y_start)
    move_z(limb, delta_z)
    right_gripper.close()
    rospy.sleep(2.0)
    move_z(limb, -delta_z)
    move_xy(limb, delta_x_piece, delta_y_piece)
    move_z(limb, delta_z)
    right_gripper.open()
    rospy.sleep(2.0)
    move_z(limb, -delta_z)



if __name__ == '__main__':
    main()
