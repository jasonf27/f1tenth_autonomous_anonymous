#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')

        # One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # You should also subscribe to the /scan topic to get the LaserScan messages and
        # the /ego_racecar/odom topic to get the current speed of the vehicle.
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)
        self.scan_subscription  # prevent unused variable warning
        self.odom_subscription

        self.speed = 0.

    def odom_callback(self, odom_msg):

        twist = odom_msg.twist.twist # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        v = twist.linear.x # Linear velocity, forward direction
        
        # Unused Odometry data, I parsed it out just to gain an understanding of what info we have
        # w = twist.angular # Vector3
        # pose = odom_msg.pose.pose
        # position = pose.position # Point
        # orientation = pose.orientation #Quaternion

        self.speed = v

        # Only for debugging purposes, not as helpful to us anymore now that things are working
        # prt = "v=" + str(v)
        # self.get_logger().info(prt)

    def scan_callback(self, scan_msg):
        
        threshold = 1.2 # Tuned this value with some trial and error, seems conservative enough
        # Yet still enables car to pass through narrow hallway in the back 
        ranges = scan_msg.ranges
        num = len(ranges)
        scan_time = scan_msg.scan_time # I didn't use, but would have for the iTTC calculation method #2,
        # namely the one where we compute speed as rate of change of LiDAR readings
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment
        v = self.speed

        for i in range(num):
            angle = angle_min + i*angle_increment # Compute angle for i'th element of ranges[]
            v_long = v * np.cos(angle) # Compute longitudinal v component i.e. in direction of this LiDAR beam
            if v_long <= 0:
                v_long = 0 # Compute {v_long}+ aka max(0,v_long), to capture risk of distance getting smaller
            else:
                iTTC = ranges[i] / v_long # Compute time to collision, using formula provided
            if v_long > 0 and iTTC < threshold: # Check if distance is decreasing, and doing so too fast
                new_msg = AckermannDriveStamped()
                new_msg.drive.speed = 0.0
                self.publisher_.publish(new_msg) # Publish Ackermann message to stop car suddenly
                prt = "EMERGENCY STOP: iTTC = " + str(iTTC) # Print to terminal to indicate what's happening
                self.get_logger().info(prt)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()