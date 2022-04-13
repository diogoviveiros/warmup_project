#!/usr/bin/env python3


# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy
import math

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to wall.
distance = 0.75

class FollowWall(object):
    """ This node walks the robot to wall and stops """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("follow_wall")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan, queue_size=1)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def process_scan(self, data):
        # Determine closeness to wall by looking at scan data from in front of
        #   the robot, set linear velocity based on that information, and
        #   publish to cmd_vel.

        # The ranges field is a list of 360 number where each number
        #   corresponds to the distance to the closest obstacle from the
        #   LiDAR at various angles. Each measurement is 1 degree apart.

        # The first entry in the ranges list corresponds with what's directly
        #   in front of the robot.
        
        
        if ( (data.ranges[0] >= distance or data.ranges[0] == 0.0)):
            # Go forward if not close enough to wall.
            
            self.twist.angular.z = 0
            self.twist.linear.x = 0.1
            
            print("Condition 2")

        elif(data.ranges[0] <= distance and (data.ranges[90] <= distance or data.ranges[90] == 0.0)):
          
            self.twist.angular.z = math.sin(math.radians(90))
            self.twist.linear.x = 0
            spin = 1
            print("Condition 3")
        
        elif(data.ranges[0] <= distance and (data.ranges[270] <= distance or data.ranges[270] == 0.0)):
            self.twist.angular.z = math.sin(math.radians(270))
            self.twist.linear.x = 0
            spin  = 1
            print("Condition 4")
        elif(data.ranges[90] == 0.0 or data.ranges[90] >= distance):
            self.twist.angular.z = math.sin(math.radians(90))
            self.twist.linear.x = 0
            spin  = 1
            print("Condition 5")
        elif(data.ranges[270] == 0.0 or data.ranges[270] >= distance):
            self.twist.angular.z = math.sin(math.radians(270))
            self.twist.linear.x = 0
            spin  = 1
            print("Condition 6")
            
        else:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0
            print("Condition else")
        

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

        if(spin == 1):
            rospy.sleep(1)
            spin = 0
            at_wall = 1


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = FollowWall()
    node.run()