#!/usr/bin/env python
# server side

import rospy
from geometry_msgs.msg import Twist

from cycu_turtlebot3.srv import Velocity, VelocityResponse

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84


class Server:
    def __init__(self):
        rospy.init_node("set_velocities_server")
        self.srv = rospy.Service("set_velocities", Velocity, self.Set_Velocities)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        print("Ready to set velocities (linear:+-0.22, angular:+-2.84):")
        rospy.spin()

    def Set_Velocities(self, request):
        print(
            "Setting velocities (linear:{0} angular:{1})".format(
                request.linear_velocity, request.angular_velocity
            )
        )

        twist = Twist()

        twist.linear.x = request.linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = request.angular_velocity

        start_time = rospy.Time.now()
        while True:
            current_time = rospy.Time.now()
            self.pub.publish(twist)

            if current_time - start_time > rospy.Duration(2):
                break

        return VelocityResponse("Velocities Set.")


if __name__ == "__main__":
    Server = Server()
