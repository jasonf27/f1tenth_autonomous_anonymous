#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

home = expanduser('~')

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger_node')
        #self.odom_sub = self.create_subscription(Odometry, 'pf/pose/odom', self.save_waypoint, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.save_waypoint, 10)
        self.file = open(strftime(home+'/615-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
        self.counter = 0

    def save_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)
        if data.twist.twist.linear.x>0.:
            print(data.twist.twist.linear.x)

        if self.counter%50 == 0:
            self.file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                        data.pose.pose.position.y,
                                        euler[2],
                                        speed))
            print('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                        data.pose.pose.position.y,
                                        euler[2],
                                        speed))
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    logger = WaypointLogger()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
