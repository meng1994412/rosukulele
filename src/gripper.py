#!/usr/bin/env python

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
SDK Gripper Example: keyboard
"""

import rospy

import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION


def main():

    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    print("Initializing node... ")
    rospy.init_node("gripper")
    s = rospy.Service('move_to', Grip, gripper)
    print("Ready to call Grip")
    rospy.spin()

def gripper(limb,bool):
    # initialize interfaces
    gripper = None
    
    try:
        gripper = intera_interface.Gripper(limb + '_gripper')
    except (ValueError, OSError) as e:
        rospy.logerr("Could not detect an electric gripper attached to the robot.")
        clean_shutdown()
        return
    rospy.on_shutdown(clean_shutdown)
    # WARNING: setting the deadzone below this can cause oscillations in
    # the gripper position. However, setting the deadzone to this
    # value is required to achieve the incremental commands in this example
    gripper.set_dead_zone(0.001)
    rospy.loginfo("Gripper deadzone set to {}".format(gripper.get_dead_zone()))
    if bool:
        gripper.close()
    else:
        gipper.open()
    # force shutdown call if caught by key handler
    rospy.signal_shutdown("Gripper moved.")


def letsgrip(myArgs):
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__,
    #                                  epilog=epilog)
    # parser.add_argument(
    #     "-l", "--limb", dest="limb", default=valid_limbs[0],
    #     choices=valid_limbs,
    #     help="Limb on which to run the gripper keyboard example"
    # )
    # parser.add_argument(
    #     "-c", "--close", type=string,
    #     nargs='+',
    #     help="close gripper")
    # parser.add_argument(
    #     "-o", "--open", type=string,
    #     nargs='+',
    #     help="open gripper")
    # args = parser.parse_args(myArgs.grip.split(" "))
    # print("To grip or not to grip")

    
    if myArgs.grip == 'c':
        gripper(valid_limbs[0],True)
    else:
        gripper(valid_limbs[0],False)



if __name__ == '__main__':
    main()


