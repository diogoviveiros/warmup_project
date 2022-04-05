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
        
        # setup the Twist message that will make the Turtlebot just stop

        stop_twist = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
        )

        #Make Robot repeat this motion four times in order to make a complete square path
        for _ in range(4):

            # allow the publisher enough time to set up before publishing the first msg
            rospy.sleep(1)

            # publish the message to move forwards, and do this for 3 seconds
            self.robot_movement_pub.publish(forward_twist)

            rospy.sleep(3)
            
            #Publish the message to make the robot stop, then wait for 1 second. 

            self.robot_movement_pub.publish(stop_twist)

            rospy.sleep(1)

            #Make the robot turn 90 degrees, then wait 1.1 seconds (slightly over 1 second to compensate for frictions and other real world effects)
            self.robot_movement_pub.publish(turn_twist)

            rospy.sleep(1.1)

            self.robot_movement_pub.publish(stop_twist)





if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()