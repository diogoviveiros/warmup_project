#!/usr/bin/env python3

import rospy
import math

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node publishes ROS messages containing the 3D coordinates of a single point """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        # setup the Twist message that will send the Turtlebot forward
        forward_twist = Twist(
            linear=Vector3(1, 0, 0),
            angular=Vector3(0, 0, 0)
        )

    # setup the Twist message that will send the Turtlebot to turn 90 degrees
        turn_twist = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, math.pi/2)
        )

        stop_twist = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
        )

        for _ in range(4):

            # allow the publisher enough time to set up before publishing the first msg
            rospy.sleep(1)

            # publish the message
            self.robot_movement_pub.publish(forward_twist)

            rospy.sleep(3)

            self.robot_movement_pub.publish(stop_twist)

            rospy.sleep(1)

            self.robot_movement_pub.publish(turn_twist)

            rospy.sleep(1.1)

            self.robot_movement_pub.publish(stop_twist)





if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()