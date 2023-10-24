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

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.declare_parameters(
        	namespace = '',
        	parameters = [
			('v', None),
			('d', None)
        	]
        )

    def timer_callback(self):
        # Can this just be done with inheritance
        # Like if Stamped extends _, then I don't need to worry about header
        # Also unsure if I'm setting these variables in the right way
        # Could I have just used YAML? Or is that for deliverable 3?
        
        v = self.get_parameter('v').get_parameter_value().double_value
        d = self.get_parameter('d').get_parameter_value().double_value
        # v = 3.0
        # d = 2.0


        sub_msg = AckermannDrive()
        sub_msg.steering_angle = d # Unsure how to intake these values
        sub_msg.speed = v
        msg = AckermannDriveStamped()
        msg.header = Header() # Unsure how to create a header, or if Empty() exists
        msg.drive = sub_msg
        
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        prt = "Publishing: v=" + str(v) + ", d=" + str(d)
        self.get_logger().info(prt)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


def main(args=None):
    
    # Haven't changed this at all, maybe should restructure
    # Must also ensure this goes as fast as possible
    # Also unsure if I'm properly linking to topics, maybe revisit early tutorial
    
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
