#!/usr/bin/env python3
# from my_cpp_py_pkg.module_to_import import ...
# from pac1 import helper.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Header

import ackermann_msgs
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('relay')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Maybe also create a new AckermannDriveStamped here and publish it?
        sub_msg = msg.drive
        d = sub_msg.steering_angle # Unsure how to intake these values
        v = sub_msg.speed
        dp = 3 * d
        vp = 3 * v
        new_msg = AckermannDriveStamped()
        new_msg.drive.steering_angle = dp
        new_msg.drive.steering_angle = vp
        self.publisher_.publish(new_msg)
        # self.get_logger().info('I heard: "%s"' % msg.data)
        prt = "Response: v=" + str(vp) + ", d=" + str(dp)
        self.get_logger().info(prt)
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
