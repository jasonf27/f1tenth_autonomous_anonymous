#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# TODO CHECK: include needed ROS msg type headers and libraries
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
# import tf
import tf2_ros
from tf_transformations import euler_from_quaternion
import csv

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.get_logger().info("Pure Pursuit Node Initialized")

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.mark1_publisher = self.create_publisher(Marker, 'goal', 10)
        self.mark2_publisher = self.create_publisher(MarkerArray, 'waypoints', 10)
        # print("pure_pursuit_node created")
        self.odom_subscription = self.create_subscription(
            Odometry, '/pf/pose/odom', self.odom_callback, 10)
        self.counter = 0
        
        self.lookahead_distance = 1.0 
        self.max_steering_angle = np.pi / 4.0  
        self.current_index = 0
        self.min_dist = float('inf')
        with open('teleop_waypoints_real_1.csv', newline='') as csvfile:
            data = np.array([list(map(float, row)) for row in csv.reader(csvfile)])

        self.waypoints = data
        #self.waypoints = np.array([[-0.3, -2.65], [3.07, 3.63], [-16.74, 14.8], [-20.92, 7.43]])

        self.kp = 1
        self.kd = 0
        self.ki = 0.00000015
        self.prev_angle = 0.0
        self.integral = 0.0
        self.prev_tmoment = self.get_clock().now()
   
        self.visualizeWaypoints()

        self.create_timer(0.2, self.waypoint_callback)

    def waypoint_callback(self):
        self.visualizeWaypoints()
        self.mark2_publisher.publish(self.markers)

    def visualizeWaypoints(self):
#        self.get_logger().info("VIsualizing Waypoints")
        markers = MarkerArray()

        t = self.get_clock().now()

        self.idx = 0
        for pt in self.waypoints:
            marker = Marker()

            marker.header.frame_id = "/map"
            marker.header.stamp = rclpy.clock.Clock().now().to_msg()
            marker.id = self.idx
            self.idx += 1

            marker.type = Marker.SPHERE
            marker.action = 0
            marker.pose.position.x = pt[0]
            marker.pose.position.y = pt[1]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1;
            marker.scale.y = 0.10;
            marker.scale.z = 0.10;

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            markers.markers.append(marker)
        self.markers = markers
        # self.mark2_publisher.publish(markers)

    def odom_callback(self, odom_msg):
        
        twist = odom_msg.twist.twist # Twist type, http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        v = twist.linear # Linear velocity, v.x and v.y and v.z
        w = twist.angular # Angular velocity, w.x and w.y and w.z

        pose = odom_msg.pose.pose # Pose type, http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
        p = pose.position # Point type, pos.x, pos.y, and pos.z
        o = pose.orientation # Quaternion type, x y z and w

        x_car = p.x
        y_car = p.y

        current_yaw = np.arctan2(
            2.0 * (o.w * o.z + 
                   o.x * o.y),
            1.0 - 2.0 * (o.y ** 2 + o.z ** 2)
        )

        # Print only every 10th value, since odom processed way too fast
        # self.counter += 1
        # if self.counter == 50:
        #     # print("odom_callback() is executing")
        #     print("x_car=" + str(round(100*x_car)/100) + "; y_car=" + str(round(100*y_car)/100))
        #     self.counter = 0

        quaternion = np.array([pose.orientation.x, 
                               pose.orientation.y, 
                               pose.orientation.z, 
                               pose.orientation.w])

        euler = euler_from_quaternion(quaternion)
        position = [x_car, y_car]
       

        point_dist =  np.sqrt(np.sum(np.square(self.waypoints[:, 0:2]-position), axis=1))
        point_index = np.where(abs(point_dist-self.lookahead_distance)< 0.2)[0]
        #print(point_index)
        for index in point_index:
            l2_0 = [self.waypoints[index, 0]-position[0], self.waypoints[index,1]-position[1]]
            goalx_veh = math.cos(euler[2])*l2_0[0] + math.sin(euler[2])*l2_0[1]
            goaly_veh = -math.sin(euler[2])*l2_0[0] + math.cos(euler[2])*l2_0[1]
            #print(goalx_veh, goaly_veh)
            #print(abs(math.atan(goalx_veh/goaly_veh)))
            if abs(math.atan(goalx_veh/goaly_veh)) <  np.pi/2 and goalx_veh>0 :
                 self.current_index = index
                 #print("point find")
                 break

        # for i in range(len(self.waypoints)):
        #     dist = np.sqrt((self.waypoints[i, 0] - x_car) ** 2 + (self.waypoints[i, 1] - y_car) ** 2)
        #     if dist < self.min_dist and i >= self.current_index:
        #         self.min_dist = dist
        #         self.current_index = i
        
        # if self.min_dist < self.lookahead_distance:
        #     self.current_index += 1
        
        # if self.current_index == len(self.waypoints):
        #     self.current_index = 0

        # Get the coordinates of the goal point in the global frame of reference
        goal_x_global = self.waypoints[self.current_index, 0]
        goal_y_global = self.waypoints[self.current_index, 1]

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rclpy.clock.Clock().now().to_msg()
        marker.id = self.idx
        self.idx += 1

        marker.type = Marker.SPHERE
        marker.action = 0
        marker.pose.position.x = goal_x_global
        marker.pose.position.y = goal_y_global
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.mark1_publisher.publish(marker)
    
        #print(self.current_index)

        # TODO: transform goal point to vehicle frame of reference
        dx = goal_x_global - x_car
        dy = goal_y_global - y_car
        goal_x = dx * np.cos(-current_yaw) - dy * np.sin(-current_yaw)
        goal_y = dx * np.sin(-current_yaw) + dy * np.cos(-current_yaw)

        # TODO: calculate curvature/steering angle
        KP_Pursuit = 0.25
        curvature = 2.0 * goal_y / (self.lookahead_distance ** 2)
        #print(curvature)
        steering_angle =  KP_Pursuit * curvature

        tmoment = self.get_clock().now()
        delta_time = 1.0 * (tmoment - self.prev_tmoment).nanoseconds / 1e9
        self.integral += self.prev_angle * delta_time
        angle_control = (self.kp * steering_angle + self.kd * (steering_angle - self.prev_angle) / delta_time + self.ki * self.integral)
        self.prev_tmoment = tmoment
        

        # TODO: publish drive message, don't forget to limit the steering angle.
        angle_control = np.clip(angle_control, -self.max_steering_angle, self.max_steering_angle)
        prev_angle = angle_control

        speed = 0.3

        drive_msg = AckermannDriveStamped()

#        drive_msg.drive.steering_angle = float(steering_angle)
#        if abs(steering_angle) > 20.0 / 180.0 * (22/7):
#            drive_msg.drive.speed = float(1.5 * speed)
#        elif abs(steering_angle) > 10.0 / 180.0 * (22/7):
#            drive_msg.drive.speed = float(1.75 * speed)

        drive_msg.drive.steering_angle = float(angle_control)
        #Irene: need to add lookup table for race speeds.
        if abs(angle_control) > 20.0 / 180.0 * (22/7):
            drive_msg.drive.speed = float(1.5 * speed)
        elif abs(angle_control) > 10.0 / 180.0 * (22/7):
            drive_msg.drive.speed = float(1.75 * speed)
        else:
            drive_msg.drive.speed = float(2.0 * speed)
        self.drive_publisher.publish(drive_msg)



def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
