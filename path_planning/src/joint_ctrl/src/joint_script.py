#! /usr/bin/env python
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse

import rospy
import numpy as np

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION


def prompt_move(side):
    limb = intera_interface.Limb(side)
    joints = limb.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        print("Executing" + str(joint_command))
        limb.set_joint_position_speed(0.3)
        limb.set_joint_positions(joint_command)

    limb.set_joint_position_speed(0.1)
    done = False
    angles = list(map(lambda x: float(x), input("Provide 7 joint angles, separated by spaces:\n").split()))
    while not done and not rospy.is_shutdown():
        limb.set_joint_positions({joint: angle for (joint, angle) in zip(joints, angles)})
        done = all([np.isclose(limb.joint_angle(joint), angle, atol=1e-02) for (joint, angle) in zip(joints, angles)])
    print(f"Done moving.")
        

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
    prompt_move(args.limb)


if __name__ == '__main__':
    main()
