#!/usr/bin/env python

import argparse
import time

import rospy
from geometry_msgs.msg import Twist

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = "-- Turtlebot3_Teleop by CYCU-ICE --"

e = """
Communications Failed
"""


class Turtlebot3_Teleop:
    def __init__(self):
        rospy.init_node("turtlebot3_teleop")
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0

    def PrintVels(self):
        print(
            "currently:\tlinear vel %s\t angular vel %s "
            % (
                self.target_linear_vel,
                self.target_angular_vel,
            )
        )

    def Constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input

        return input

    def CheckLinearLimitVelocity(self, vel):
        vel = self.Constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        return vel

    def CheckAngularLimitVelocity(self, vel):
        vel = self.Constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        return vel

    def Set_Linear_Velocity(self, target_linear_vel):
        self.target_linear_vel = self.CheckLinearLimitVelocity(
            target_linear_vel + LIN_VEL_STEP_SIZE
        )
        self.PublishTopic()

    def Set_Angular_Velocity(self, target_angular_vel):
        self.arget_angular_vel = self.CheckAngularLimitVelocity(
            target_angular_vel + ANG_VEL_STEP_SIZE
        )
        self.PublishTopic()

    def Reset_All_Velocity(self):
        self.target_linear_vel = 0.0
        self.control_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_angular_vel = 0.0
        self.PublishTopic()

    def PublishTopic(self):
        twist = Twist()

        twist.linear.x = self.target_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.target_linear_vel

        self.PrintVels()
        self.pub.publish(twist)


if __name__ == "__main__":
    turtlebot3_model = rospy.get_param("model", "burger")
    teleop = Turtlebot3_Teleop()
    print(msg)

    try:
        while not rospy.is_shutdown():
            # here are some examples
            teleop.Set_Linear_Velocity(1.0)
            # use sleep as a timer to maintain the speed
            time.sleep(5)
            teleop.Reset_All_Velocity()

    except:
        print(e)
