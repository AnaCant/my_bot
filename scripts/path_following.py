#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

import numpy as np
from math import sin, cos, pi
import math
import socket


class PathFollowing(Node):

    last_pose = np.array([0, 0, 0])
    current_pose = np.array([0, 0, 0])
    destinations = [[0.0, 0.0], [-0.268, 1.0], [-1.0, 1.732], [-2.0, 2.0], [-3.0, 1.732], [-3.7320, 1.0], [-4.0, 0.0], [-3.7320, -1.0], [-3.0, -1.732], [-2.0, -2.0], [-1.0, -1.732], [-0.268, -1.0], [0.0, 0.0]]  # list of destinations for the vehicle to follow
    current_destination_index = 0

    def __init__(self):
        super().__init__('path_following')
        self.subscription = self.create_subscription(Odometry, 'odom', self.control_callback, 10) # (type of message, topic, function that is activated, 10Hz)
        self.subscription  # prevent unused variable warning 

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    def distance_to_destination(self):
        """Calculate the distance between the current position and the current destination."""
        dx = self.current_pose[0] - self.destinations[self.current_destination_index][0]
        dy = self.current_pose[1] - self.destinations[self.current_destination_index][1]
        return math.sqrt(dx**2 + dy**2) # calculate euclidean distance

    def control_callback(self, msg):
        """Control law to go to a series of points/locations 
            maintaining a constant forward speed u.linear.x = 0"""
        K = 2.0 # static gain

        self.last_pose = self.current_pose
        roll, pitch, yaw = euler_from_quaternion(msg.pose.pose.orientation) # convert orientation in quaternion to euler
        self.current_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]) # reduced representation of the pose [x,y,yaw] [0,1,2]

        # control law
        u = Twist()
        u.linear.x = 1.0 # fija m/s
        distance = self.distance_to_destination()
        if distance > 0.3: # if distance to destination is greater than a small threshold
            # steer towards the current destination
            yaw_to_destination = math.atan2(self.destinations[self.current_destination_index][1] - self.current_pose[1], self.destinations[self.current_destination_index][0] - self.current_pose[0])
            u.angular.z = K * math.sin(yaw_to_destination - self.current_pose[2])
        else:
            # switch to the next destination when we reach the current destination
            if self.current_destination_index == len(self.destinations) - 1:  # if it's the last destination
                u.linear.x = 0.0
                u.angular.z = 0.0
            else: # update the current_destination_index with the next destination in the destinations list
                self.current_destination_index = (self.current_destination_index + 1) % len(self.destinations)

        # publish the control signal on the cmd_vel topic
        self.publisher_.publish(u)
        self.get_logger().info('Publishing: "%s"' % u)
        
def euler_from_quaternion(q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def main(args=None):
    rclpy.init(args=args)

    path_following = PathFollowing()

    rclpy.spin(path_following)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_following.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
