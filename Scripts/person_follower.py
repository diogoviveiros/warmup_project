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

# How close we will get to the person.
distance = 0.4

class FollowPerson(object):
    

    def __init__(self):
        # Start rospy node.
        rospy.init_node("follow_person")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan, queue_size =1)

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

        min_dist = math.inf
        angle = 0

        #Checks the angle in LIDAR which has the smallest distance
        for i in range(360):
            if(data.ranges[i] <= min_dist and data.ranges[i] != 0.0):
                min_dist = data.ranges[i]
                angle = i

         
        #Rotate in direction where the distance is lowest (determined above)
        self.twist.angular.z = math.sin(math.radians(angle))
        
            
        self.twist_pub.publish(self.twist)
        
            
        #If the robot has an obstacle directly ahead, stop. Otherwise, keep moving forwards!
        if(data.ranges[0] == 0.0 or data.ranges[0]
         >= distance):
            self.twist.linear.x = 0.1
        else:
            self.twist.linear.x = 0.0


        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)
        
        


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = FollowPerson()
    node.run()
