#!/usr/bin/env python
# client side

import argparse
import sys
import traceback

import rospy
from ros_turtlebot3_teleop.srv import Velocity, VelocityResponse


def Set_Velocities_Client(linear_velocity, angular_velocity):
    rospy.wait_for_service("set_velocities")
    try:
        set_velocities = rospy.ServiceProxy("set_velocities", Velocity)
        response = set_velocities(linear_velocity, angular_velocity)
        print(response)
    except Exception:
        traceback.print_exc()


if __name__ == "__main__":
    # add arguments
    parser = argparse.ArgumentParser(description="Turtlebot3_Teleop Client Script.")
    parser.add_argument(
        "--linear", type=float, required=True, help="Linear velocity: +-0.22"
    )
    parser.add_argument(
        "--angular", type=float, required=True, help="Angular velocity: +-2.84"
    )
    arg = parser.parse_args()

    if -0.22 <= arg.linear <= 0.22 and -2.84 <= arg.angular <= 2.84:
        print("Requesting (linear:{0}, angular:{1})".format(arg.linear, arg.angular))
        Set_Velocities_Client(arg.linear, arg.angular)
    else:
        print("Wrong velocity values.")
