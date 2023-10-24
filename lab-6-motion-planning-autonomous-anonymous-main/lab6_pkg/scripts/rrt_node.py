#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

# TODO: import as you need
import random
from matplotlib import pyplot as plt
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion
import csv

# class def for tree nodes
# It's up to you if you want to use this
class TreeNode(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        # self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        super().__init__('rrt_node')

        # TOPIC NAMES
        odom_topic = "/ego_racecar/odom"
        # odom_topic = "/pf/pose/odom"
        scan_topic = "/scan"

        occ_topic = "/occ_map"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # DESIGN VARIABLES, TO BE TUNED??
        # Occupancy grid parameters
        self.occ_height = 5.0   #meters
        self.occ_width = 3.0    #meters
        self.resolution = 0.05  #meters
        # RRT path planning
        self.lookahead_distance = 2.0   #meters      TOGGLE THIS?
        self.max_dist_from_parent = 0.5     #meters  TOGGLE THIS
        self.neighborhood_rad = 0.8     #meters
        self.goal_thresh = 0.30         #meters         TOGGLE THIS
        self.MAX_ITER = 30            #iterations     TOGGLE THIS, first 1000 then 20, now 30
        # Path following
        self.speed = 0.5                #m/s            TOGGLE THIS
        self.current_rrt_goal_index = 1                 # TOGGLE THIS?
        # Jason was confused by this ^, it seems to be hardcoded (to 1) again in odom_callback()?
        # Controls parameters
        self.kp = 0.15                             # TOGGLE THIS
        self.kd = 0
        self.ki = 0   # Jason changed this from 0.00000015 to ensure no tiny hard-to-track bad impact
        # Physical geometry
        self.car_width = 0.30       # meters

        # TODO: create subscribers
        self.odom_sub_ = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        
        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.occ_pub_ = self.create_publisher(OccupancyGrid, occ_topic, 10)
        self.mark1_publisher = self.create_publisher(Marker, 'goal', 10)
        self.path_publisher = self.create_publisher(Path, 'path', 10)

        # class attributes
        # TODO: maybe create your occupancy grid here
        self.occ_size = np.array([self.occ_height, self.occ_width]) 
        self.occ = np.zeros((int(self.occ_size[0] / self.resolution), int(self.occ_size[1] / self.resolution)), dtype=float)
        with open('teleop_waypoints_sim_1.csv', newline='') as csvfile:
            data = np.array([list(map(float, row)) for row in csv.reader(csvfile)])

        self.waypoints = data
        self.max_steering_angle = np.pi / 4.0
        self.current_index = 0

        self.odom_counter = 0
        self.path_following = False
        self.rrt_path = None
        self.local_goal_x = 0
        self.local_goal_y = 0

        # Initialize self.drive_msg here, globally, instead of odom_callback(), to ensure continuity between odom_callback() calls
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.steering_angle = 0.0
        self.drive_msg.drive.speed = 0.0

        self.prev_angle = 0.0
        self.integral = 0.0
        self.prev_tmoment = self.get_clock().now()
        self.idx = 0


    def map_to_index(self, x, y):
        i = int(np.floor((x + self.occ_size[0] / 2) / self.resolution))
        j = int(np.floor((-y + self.occ_size[1] / 2) / self.resolution))
        return i, j
    
    def isInside(self, x, y):
        return (-self.occ_size[0] / 2 <= x <= self.occ_size[0] / 2) and (-self.occ_size[1] / 2 <= y <= self.occ_size[1] / 2) 

    def is_free(self, x, y):
        x_idx, y_idx = self.map_to_index(x, y)

        # Check if the cell is within the grid bounds
        if x_idx >= 0 and x_idx < self.occ.shape[0] and y_idx >= 0 and y_idx < self.occ.shape[1]:
            # Check if the cell is free
            if self.occ[x_idx, y_idx] == 1:
                return False
        # return False

        # Jason: Checking 3 additional grid cells in each direction means 15cm = 6" = 0.5', seems about right
        cells_extra = math.ceil(self.car_width / 2 / self.resolution)
        for i in range(1, cells_extra+1):
            if y_idx-i >= 0 and self.occ[x_idx][y_idx-i]:
                return False
            if y_idx+i < self.occ_width / self.resolution and self.occ[x_idx][y_idx+i]:
                return False
            
        return True

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        angle = scan_msg.angle_min
        max_angle = scan_msg.angle_max

        tolerance = 0.02
        for dis in scan_msg.ranges:

            d = 0
            while d < dis:
                x = d * np.cos(angle)
                y = d * np.sin(angle)

                if not self.isInside(x, y):
                    break

                i, j = self.map_to_index(x, y)

                self.occ[i, j] = 0

                d += self.resolution

            while True:

                x = d * np.cos(angle)
                y = d * np.sin(angle)

                if not self.isInside(x, y):
                    break

                i, j = self.map_to_index(x, y)

                self.occ[i, j] = 1

                d += self.resolution

            angle += scan_msg.angle_increment

        occ_msg = OccupancyGrid()
        occ_msg.header.frame_id = "ego_racecar/base_link"
        occ_msg.header.stamp = self.get_clock().now().to_msg()

        occ_msg.info.resolution = self.resolution
        occ_msg.info.width = int(self.occ_size[1] / self.resolution)
        occ_msg.info.height = int(self.occ_size[0] / self.resolution)
        
        occ_msg.info.origin.position.x = -self.occ_size[0] / 2.0
        occ_msg.info.origin.position.y = self.occ_size[1] / 2.0  
        occ_msg.info.origin.orientation.x = 0.0  
        occ_msg.info.origin.orientation.y = 0.0  
        occ_msg.info.origin.orientation.z = -0.7071  
        occ_msg.info.origin.orientation.w = 0.7071

        probability = self.occ.flatten() * 100
        probability = np.int8(probability).tolist()
        occ_msg.data = probability

        self.occ_pub_.publish(occ_msg)

        # image = np.flip(self.occ, 0)
        # plt.imshow(image, interpolation='nearest')
        # plt.show()
        # plt.close()

    def odom_callback(self, odom_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # self.odom_counter += 1
        # if self.odom_counter != 10:
        #     return
        # self.odom_counter = 0

        #print(self.map_to_index(2.5, 1.0))
        #print(self.map_to_index(self.index_to_map(0, 0)[0], self.index_to_map(0, 0)[1]))
        #assert self.map_to_index(self.index_to_map(0, 0)[0], self.index_to_map(0, 0)[1]) == (0, 0)

        if self.drive_msg.drive.steering_angle != 0 or self.drive_msg.drive.speed != 0:
            self.drive_pub_.publish(self.drive_msg)
            # pass
        


        # TODO Figure out how to follow this path "path"
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

        quaternion = np.array([pose.orientation.x, 
                               pose.orientation.y, 
                               pose.orientation.z, 
                               pose.orientation.w])

        euler = euler_from_quaternion(quaternion)
        position = [x_car, y_car]

        if self.rrt_path:
            if self.current_rrt_goal_index < len(self.rrt_path):
                rrt_goal_x = self.rrt_path[self.current_rrt_goal_index].x
                rrt_goal_y = self.rrt_path[self.current_rrt_goal_index].y
                #if np.sqrt((rrt_goal_x - x_car) ** 2 + (rrt_goal_y - y_car) ** 2) < 0.05:
                self.current_rrt_goal_index += 1
            
            if self.current_rrt_goal_index > min(10, len(self.rrt_path)-1):
                self.path_following = False

        if not self.path_following:
            # print("Generating new RRT Path")
            point_dist =  np.sqrt(np.sum(np.square(self.waypoints[:, 0:2]-position), axis=1))
            point_index = np.where(abs(point_dist-self.lookahead_distance)< 0.2)[0]

            for index in point_index:
                l2_0 = [self.waypoints[index, 0]-position[0], self.waypoints[index,1]-position[1]]
                goalx_veh = math.cos(euler[2])*l2_0[0] + math.sin(euler[2])*l2_0[1]
                goaly_veh = -math.sin(euler[2])*l2_0[0] + math.cos(euler[2])*l2_0[1]
                if abs(math.atan(goalx_veh/goaly_veh)) <  np.pi/2 and goalx_veh>0 :
                    self.current_index = index
                    break

            # Get the coordinates of the goal point in the global frame of reference
            self.goal_x_global = self.waypoints[self.current_index, 0]
            self.goal_y_global = self.waypoints[self.current_index, 1]

            # TODO: transform goal point to vehicle frame of reference
            self.local_goal_x, self.local_goal_y = self.global_to_local(self.goal_x_global, self.goal_y_global, x_car, y_car, current_yaw)

            if not self.isInside(self.local_goal_x, self.local_goal_y) or not self.is_free(self.local_goal_x, self.local_goal_y):
                self.local_goal_x, self.local_goal_y = self.constrain_within_occ(self.local_goal_x, self.local_goal_y, self.occ_size[0], self.occ_size[1])

            # print("Local goal: ", self.local_goal_x, self.local_goal_y)
            assert self.isInside(self.local_goal_x, self.local_goal_y)
            assert self.is_free(self.local_goal_x, self.local_goal_y)

            root_node = TreeNode()
            root_node.cost = 0
            root_node.x = 0
            root_node.y = 0

            tree = [root_node]
            
            i = 0
            while i in range(self.MAX_ITER):
            # for i in range(self.MAX_ITER):
                (x_samp, y_samp) = self.sample()
                i_nearest = self.nearest(tree, (x_samp, y_samp))
                # print("Sampling loop:", i)
                nearest_node = tree[i_nearest]
                new_node = self.steer(nearest_node, (x_samp, y_samp))
                if (not self.check_collision(nearest_node, new_node)):
                    
                    # RRT* ADDITIONS GO HERE LINE 285

                    neighborhood = self.near(tree, new_node)
                    min_node = nearest_node
                    new_node.cost = nearest_node.cost + self.line_cost(new_node, nearest_node)
                    for near_node in neighborhood:
                        cond1 = not self.check_collision(near_node, new_node)
                        trial_cost = near_node.cost + self.line_cost(new_node, near_node)
                        cond2 = trial_cost < new_node.cost
                        # cond3 = near_node != new_node
                        if (cond1 and cond2):
                            min_node = near_node
                            new_node.cost = trial_cost
                    new_node.parent = min_node
                    for near_node in neighborhood:
                        cond1 = not self.check_collision(near_node, new_node)
                        trial_cost = new_node.cost + self.line_cost(new_node, near_node)
                        cond2 = trial_cost < near_node.cost
                        if (cond1 and cond2):
                            near_node.cost = trial_cost
                            near_node.parent = new_node
                    
                    tree.append(new_node)



                    # RRT BEFORE ADDING THE *
                    
                    # tree.append(new_node)


                    self.rrt_path = self.find_path(tree, new_node)
                    #print("path len", len(tree))
                    if (self.rrt_path):
                        for i in range(len(self.rrt_path)):
                            self.rrt_path[i].x, self.rrt_path[i].y = self.local_to_global(self.rrt_path[i].x, self.rrt_path[i].y, x_car, y_car, current_yaw)
                        break
                i += 1

            if i == self.MAX_ITER:
                # print("MAX_ITER WAS REACHED")
                return # Jason added this since nothing can/should happen if no path was found
            
            # Jason commented out this below, Wednesday night while testing,
            # b/c he's scared it would prevent execution of the RRT loop of future odom_callback() calls
            self.path_following = True

            self.current_rrt_goal_index = 1
            # print("RRT Path created")
            path_msg = Path()
            path_msg.header.frame_id = 'map'

            for point in self.rrt_path:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = point.x
                pose.pose.position.y = point.y
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            
            path_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.path_publisher.publish(path_msg)
            self.get_logger().info('Publishing path')

        # print("Following existing RRT Path Node: ", self.current_rrt_goal_index)
        # print("self.rrt_path =", self.rrt_path)
        rrt_goal_x = self.rrt_path[self.current_rrt_goal_index].x
        rrt_goal_y = self.rrt_path[self.current_rrt_goal_index].y
        # print("RRT_goal", rrt_goal_x, rrt_goal_y)
        rrt_goal_x_local, rrt_goal_y_local = self.global_to_local(rrt_goal_x, rrt_goal_y, x_car, y_car, current_yaw)


        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rclpy.clock.Clock().now().to_msg()
        marker.id = self.idx
        self.idx += 1

        marker.type = Marker.SPHERE
        marker.action = 0
        # marker.pose.position.x = rrt_goal_x
        # marker.pose.position.y = rrt_goal_y
        marker.pose.position.x = self.goal_x_global
        marker.pose.position.y = self.goal_y_global
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.mark1_publisher.publish(marker)
    
        # print(self.current_index)

        # TODO: calculate curvature/steering angle
        curvature = 2.0 * rrt_goal_y_local / (rrt_goal_x_local ** 2)
        # print(curvature)
        # KP_Pursuit = 0.15
        steering_angle = curvature # *KP_Pursuit
        # Jason replaced KP_Pursuit with previously unused self.kp, see below
        # Since Kd and Ki are both 0, this is simpler, yet mathematically equivalent to before

        tmoment = self.get_clock().now()
        delta_time = 1.0 * (tmoment - self.prev_tmoment).nanoseconds / 1e9
        self.integral += self.prev_angle * delta_time
        angle_control = (self.kp * steering_angle + self.kd * (steering_angle - self.prev_angle) / delta_time + self.ki * self.integral)
        self.prev_tmoment = tmoment
        

        # TODO: publish drive message, don't forget to limit the steering angle.
        angle_control = np.clip(angle_control, -self.max_steering_angle, self.max_steering_angle)
        self.prev_angle = angle_control

#        self.drive_msg.drive.steering_angle = float(steering_angle)
#        if abs(steering_angle) > 20.0 / 180.0 * (22/7):
#            self.drive_msg.drive.speed = float(1.5 * self.speed)
#        elif abs(steering_angle) > 10.0 / 180.0 * (22/7):
#            self.drive_msg.drive.speed = float(1.75 * self.speed)


        # Note: Jason made drive_msg variable global, so it can continuously maintain value between odom_callback() calls
        # without accidentally halting the car while RRT loop runs for finite nonzero time
        self.drive_msg.drive.steering_angle = float(angle_control)
        #Irene: need to add lookup table for race speeds.
        if abs(angle_control) > 20.0 / 180.0 * (22/7):
            self.drive_msg.drive.speed = float(1.5 * self.speed)
        elif abs(angle_control) > 10.0 / 180.0 * (22/7):
            self.drive_msg.drive.speed = float(1.75 * self.speed)
        else:
            self.drive_msg.drive.speed = float(2.0 * self.speed)
        
        # print("Steering: ", angle_control)
        self.drive_pub_.publish(self.drive_msg)

        return None

    def global_to_local(self, goal_x_global, goal_y_global, x_car, y_car, current_yaw):
        dx = goal_x_global - x_car
        dy = goal_y_global - y_car
        goal_x = dx * np.cos(-current_yaw) - dy * np.sin(-current_yaw)
        goal_y = dx * np.sin(-current_yaw) + dy * np.cos(-current_yaw)
        
        return goal_x, goal_y

    def local_to_global(self, local_x, local_y, x_car, y_car, current_yaw):
        car_pos_global = np.array([x_car, y_car])
        point_local = np.array([local_x, local_y])

        R_car_global = np.array([[np.cos(current_yaw), -np.sin(current_yaw)],
                    [np.sin(current_yaw), np.cos(current_yaw)]])
        
        points_rotated = np.matmul(R_car_global, point_local)
        points_global = points_rotated + car_pos_global
    
        return points_global
    
    def constrain_within_occ(self, x, y, height, width):
        if (abs(x) < height / 2 and abs(y) < width / 2):
            x_new = x
            y_new = y
        elif (x == 0):
            x_new = 0
            y_new = width / 2 * y / abs(y)
        elif (y == 0):
            x_new = height / 2 * x / abs(x)
            y_new = 0
        elif (abs(y/x) > abs(width/height)):
            y_new = width / 2 * abs(y) / y
            x_new = x * width / 2 / abs(y)
        else:
            x_new = height / 2 * abs(x) / x
            y_new = y * height / 2 / abs(x)
        #if not self.isInside(x_new, y_new):

        x_new = np.clip(x_new, -height/2, height/2)
        y_new = np.clip(y_new, -width/2, width/2)
        

        # ENSURE NOT IN AN OBSTACLE
        magn0 = np.sqrt(x_new ** 2 + y_new ** 2)
        x_norm = x_new / magn0
        y_norm = y_new / magn0
        
        x_step = self.resolution * x_norm
        y_step = self.resolution * y_norm

        # print("CONSTRAINING")
        
        while not self.is_free(x_new, y_new):
            x_new -= x_step
            y_new -= y_step
            if abs(x_new) > height/2 or abs(y_new) > width/2:
                # print("out of bounds!")
                break
        assert self.is_free(x_new, y_new)
        
        x_maybe = x_new - x_norm * self.goal_thresh
        y_maybe = y_new - y_norm * self.goal_thresh

        if self.is_free(x_maybe, y_maybe):
            (x_new, y_new) = (x_maybe, y_maybe)

        return (x_new, y_new)

    
    # helper by Irene (need to check)
    def get_min_max_x_y(self,):
        min_x, max_x, min_y, max_y = float('inf'), -float('inf'), float('inf'), -float('inf')

        for i in range(len(self.occ)):
            for j in range(len(self.occ[0])):
                if self.occ[i][j] == 1:
                    x, y = self.index_to_map(i, j)
                    min_x = min(min_x, x)
                    max_x = max(max_x, x)
                    min_y = min(min_y, y)
                    max_y = max(max_y, y)
        
        return min_x, max_x, min_y, max_y
    
    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = None
        y = None

        sample_iter = 0
        while sample_iter < 100:
            # randomly sample a point within the map boundaries
            # min_x, max_x, min_y, max_y = self.get_min_max_x_y()
            # print("entered sample loop: ", sample_iter)
            (length, width) = (self.occ_size[0], self.occ_size[1])
            x = random.uniform(0, length/2)
            y = random.uniform(-width/2, width/2)
            #print("x, y", x, y)
            sample_iter = sample_iter + 1

            # check if the sampled point is in free space
            if self.is_free(x, y):
                return (x, y)
        # print("sample() reached max of sample_iter = 100")

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        min_distance = np.inf

        for i in range(len(tree)):
            node = tree[i]
            distance = np.sqrt((sampled_point[0] - node.x)**2 + (sampled_point[1] - node.y)**2)
            if distance < min_distance:
                nearest_node = i
                min_distance = distance 
        
        return nearest_node


    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """

        (x_samp, y_samp) = sampled_point
        (x_near, y_near) = (nearest_node.x, nearest_node.y)
        r = self.max_dist_from_parent
        x_del = x_samp - x_near
        y_del = y_samp - y_near
        th = math.atan2(y_del, x_del)
        dist = math.sqrt(y_del ** 2 + x_del ** 2)
        new_node = TreeNode()
        if(r > dist):
            (x_new, y_new) = (x_samp, y_samp)
        else:
            x_new = x_near + r * math.cos(th)
            y_new = y_near + r * math.sin(th)
        new_node.x = x_new
        new_node.y = y_new
        new_node.parent = nearest_node

        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        dx = new_node.x - nearest_node.x
        dy = new_node.y - nearest_node.y
        dist = math.sqrt(dx*dx + dy*dy)
        num_samples = int(math.ceil(dist / self.resolution))
        x_samples = np.linspace(nearest_node.x, new_node.x, num_samples)
        y_samples = np.linspace(nearest_node.y, new_node.y, num_samples)

        for i in range(num_samples):
            x = x_samples[i]
            y = y_samples[i]
            xi, yi = self.map_to_index(x, y)
            if self.occ[xi][yi]:
                return True
            

            # Checking 3 additional grid cells in each direction means 15cm = 6" = 0.5', seems about right
            cells_extra = math.ceil(self.car_width / 2 / self.resolution)
            for i in range(1, cells_extra+1):
                if yi-i >= 0 and self.occ[xi][yi-i]:
                    return True
                if yi+i < self.occ_width / self.resolution and self.occ[xi][yi+i]:
                    return True

        return False
    
    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enough to the goal
        """
        dx = latest_added_node.x - goal_x
        dy = latest_added_node.y - goal_y
        dist = math.sqrt(dx*dx + dy*dy)
        #IRENE: change this is needed
       
        return dist < self.goal_thresh

    def euclidean_distance(self, node1, node2):
        dx = node1.x - node2.x
        dy = node1.y - node2.y
        dist = math.sqrt(dx*dx + dy*dy)
        return dist

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """

        if self.is_goal(latest_added_node, self.local_goal_x, self.local_goal_y):
            path = [latest_added_node]
            while path[-1].parent is not None:
                path.append(path[-1].parent)
            path.reverse()

            return path

        path = []
        return path

    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
    
        if node.parent is None:
            return 0

        if node.parent.parent is None:
            return self.line_cost(node, node.parent)

        return node.parent.cost + self.line_cost(node, node.parent)

        # cost = 0
        # while node.parent is not None:
        #     cost += self.line_cost(node.parent, node)
        #     node = node.parent
        # return cost

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return ((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2) ** 0.5

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []

        for n in tree:
            #IRENE: Can change the r value
            r = self.neighborhood_rad
            if math.sqrt((n.x - node.x)**2 + (n.y - node.y)**2) <= r:
                neighborhood.append(n)
        return neighborhood

def main(args=None):
    rclpy.init(args=args)
    # print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()