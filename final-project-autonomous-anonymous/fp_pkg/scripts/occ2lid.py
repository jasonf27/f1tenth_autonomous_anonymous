#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from numpy import linalg as LA
import math

from sensor_msgs.msg import LaserScan, Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

import cv2
import time
import os

from cv_bridge import CvBridge, CvBridgeError

#cap = cv2.VideoCapture(4)
#cap.set(3, 960)
#cap.set(4, 540)

fx, fy = 693.67797621, 694.76838489  # focal lengths
cx, cy = 448.53498348, 258.05020739  # optical center
camera_height = 0.12  # height from the ground
pitch_angle = np.pi / 2 + 0.55  # pitch angle in radians
yaw_angle = 0.08  # yaw angle in radians
# Rotation matrix for the yaw angle
R_yaw = np.array([[np.cos(yaw_angle), -np.sin(yaw_angle)],
                  [np.sin(yaw_angle), np.cos(yaw_angle)]])

grid_size = 0.01  # size of each grid cell in meters
grid_width = 2.0  # width of the occupancy grid in meters
grid_height = 2.0  # height of the occupancy grid in meters
angle_increment = 0.004  # angle increment in radians
range_max = 30.0 # maximum range of the laser scanner in meters
range_min = 0.0 # minimum range of the laser scanner in meters

def draw_green_edges(image, area_threshold=20):
    # Check if the image is not empty
    if image is None:
        print(f"Error: Unable to read the image. Please check the file path or format.")
        return

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the yellow color in HSV space
    lower_yellow = np.array([20, 1, 180])
    upper_yellow = np.array([36, 255, 255])

    # Create a mask for yellow color
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Apply the mask to the original image
    masked_image = cv2.bitwise_and(image, image, mask=mask)

    # Convert the masked image to grayscale
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to remove noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    _, binary_img = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)

    binary_img[:int(binary_img.shape[0]/2), :] = 0


    # # Detect edges using Canny edge detector
    # edges = cv2.Canny(blurred, 50, 200)

    # Dilate the edges to close gaps in the detected lines
    kernel = np.ones((2, 2), np.uint8)
    dilated = cv2.dilate(blurred, kernel, iterations=1)

    # # Find contours of the dilated edges
    # contours, _ = cv2.findContours(dilated_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # filtered_contours = [contour for contour in contours if cv2.contourArea(contour) >= area_threshold]


    # # Draw green contours on the original image
    # green_edges_image = cv2.drawContours(image.copy(), filtered_contours, -1, (0, 255, 0), 2)

    # Save the result
    # output_image_path = "green_edges.png"
    # cv2.imwrite(output_image_path, green_edges_image)

    return binary_img

def get_occ_grid(img):
    height, width = img.shape
    # Create the occupancy grid (initialize as all free space)
    grid = np.zeros((int(grid_height / grid_size), int(grid_width / grid_size)))

    # Create 2D arrays for pixel coordinates
    u, v = np.meshgrid(np.arange(width), np.arange(height))

    # Identify the occupied pixels
    occupied_pixels = img == 255

    # Estimate the depth (Z) for each pixel
    Z = camera_height / np.cos(pitch_angle - np.arctan((v - cy) / fy))

    # Calculate the real-world coordinates (X, Y) for each pixel
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    # Calculate the grid cell coordinates
    grid_x = (X / grid_size).astype(int)
    grid_y = (Y / grid_size).astype(int)

    # Update the occupancy grid
    valid = (0 <= grid_x) & (grid_x < grid.shape[1]) & (0 <= grid_y) & (grid_y < grid.shape[0])
    grid[grid_y[valid & occupied_pixels], grid_x[valid & occupied_pixels]] = 1

    return grid

def get_occ_grid_alt(img):
    #print(img.shape)

    #for x in range(width):
    #    for y in range(height):
    #        if img[y, x] == 255:  # if the pixel is occupied
    #            x_car = fy *(camera_height - 0)/ (y - cy)
    #            y_car = -x_car*(x - cx)/fx

    #            angle = np.arctan2(y_car, x_car)
    #            distance = np.sqrt(x_car**2 + y_car**2)
    #            angles.append([angle, distance])
    
    y_coords, x_coords = np.where(img == 255)
    if y_coords.shape[0] == 0:
        return None, None, None, None, None, None 
    theta = pitch_angle + np.arctan((y_coords - cy) / fy)
    Z = camera_height / -np.cos(theta)
    x_cam = Z * np.sin(theta) - 0.725
    y_cam = -(x_coords - cx) * Z / fx + 0.033
    # Rotate the X_cam and Y_cam coordinates
    x_car, y_car = np.dot(R_yaw, np.array([x_cam, y_cam]))
    #x_car = fy * (camera_height - 0) / (y_coords - cy)
    #y_car = -x_car * (x_coords - cx) / fx
    angle = np.arctan2(y_car, x_car)
    distance = np.sqrt(x_car ** 2 + y_car ** 2)

    angles = np.stack((angle, distance), axis=-1)

    # Convert the data into a structured numpy array for ease of sorting
    dtype = [('angle', float), ('distance', float)]
    structured_data = np.array(list(zip(angles[:, 0], angles[:, 1])), dtype=dtype)

    # Sort the structured array by angle
    sorted_data = np.sort(structured_data, order='angle')

    # Round the angles to the nearest hundredth
    rounded_angles = np.round(sorted_data['angle'] / angle_increment) * angle_increment

    # Find the unique rounded angles and their indices
    unique_angles, indices = np.unique(rounded_angles, return_index=True)

    # For each unique rounded angle, find the smallest distance
    min_distances = np.minimum.reduceat(sorted_data['distance'], indices)

    # Generate an array for all possible angles from min to max with the given increment
    # all_angles = np.arange(rounded_angles.min(), rounded_angles.max() + angle_increment, angle_increment)
    all_angles = np.arange(-2.360, 2.3600 + angle_increment, angle_increment)
    print(all_angles[-5:])
    # all_angles = np.round(all_angles, 2)  # Ensure that the rounding matches
    all_angles = np.round(all_angles / angle_increment) * angle_increment

    # Create an array for the distances with a default value
    default_value = range_max  # Set this to your desired default value
    all_distances = np.full(all_angles.shape, default_value)

    # Use numpy indexing to insert the min_distances into the all_distances array
    np.put(all_distances, np.searchsorted(all_angles, unique_angles), min_distances)

    # Create the final output array
    output = np.column_stack((all_angles, all_distances))

    # To get the minimum angle, maximum angle, and angle increment
    min_angle = np.min(all_angles)
    max_angle = np.max(all_angles)
    # angle_increment = np.diff(all_angles).mean()  # This assumes the angle increment is roughly constant

    print('Minimum Angle: ', min_angle)
    print('Maximum Angle: ', max_angle)
    print('Angle Increment: ', angle_increment)
    # print('Minimum Range: ', np.min(all_distances))
    # print('Maximum Range: ', np.max(all_distances))
    print(output.shape)
    print(all_angles[:5])

    return min_angle, max_angle, angle_increment, range_min, range_max, all_distances

class OccupancyGridToLiDAR(Node):


    def __init__(self, onboard=True):
        super().__init__('occ2lid_node')

        # LiDAR subscriber
        scan_topic = '/scan'
#        self.scan_sub_ = self.create_subscription(
#            LaserScan,
#            scan_topic,
#            self.scan_callback,
#            10)
        
        # Fake LiDAR publisher
        fake_lidar_topic = '/fake_scan'
        self.fake_lidar_pub_ = self.create_publisher(LaserScan, fake_lidar_topic, 10)
        self.FL_angle_min = 0 # Declared here, initialized and given a value later
        self.FL_angle_max = 0 # Declared here, initialized and given a value later
        self.FL_angle_increment = 0 # Declared here, initialized and given a value later

        self.FL_range_max = 30.0
        self.FL_range_min = 0.0

        # Occupancy grid publisher
        occ_topic = "/occ_map"
        self.occ_pub_ = self.create_publisher(OccupancyGrid, occ_topic, 10)

        # Occupancy grid parameters and initialization
        # TODO Update this stuff once OccGrid generated from Camera, not LiDAR
        self.occ_height = 2.0#5.0   #meters
        self.occ_width = 2.0#3.0    #meters
        self.resolution = 0.01  #meters
        self.occ_size = np.array([self.occ_height, self.occ_width]) 
        self.occ = np.zeros((int(self.occ_size[0] / self.resolution), int(self.occ_size[1] / self.resolution)), dtype=float)

        self.cap = cv2.VideoCapture(4)
        self.cap.set(3, 960)
        self.cap.set(4, 540)

        self.bridge = CvBridge()

        if onboard:
            self.img_pub_ = self.create_publisher(Image, "/frames", 10)
        else:
            self.img_sub_ = self.create_subscription(Image, '/frames', self.image_callback, 10)

        time_period = 0.02
        self.timer = self.create_timer(time_period, self.timer_callback)

    def image_callback(self, image):
        frame = self.bridge.imgmsg_to_cv2(image)

        img = draw_green_edges(frame)

        self.img_pub_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            # cv2.imshow("image", img)
            #self.occ = get_occ_grid(img)
        min_angle, max_angle, angle_increment, range_min, range_max, ranges = get_occ_grid_alt(img)

        if min_angle is None:
            return

        fake_lidar_msg = LaserScan()
        fake_lidar_msg.ranges = ranges.tolist()
        fake_lidar_msg.angle_min = min_angle
        fake_lidar_msg.angle_max = max_angle
        fake_lidar_msg.angle_increment = angle_increment
        fake_lidar_msg.range_min = range_min
        fake_lidar_msg.range_max = range_max

        # fake_lidar_msg.header.frame_id = "ego_racecar/base_link"
        fake_lidar_msg.header.frame_id = "laser"
        fake_lidar_msg.header.stamp = self.get_clock().now().to_msg()

        self.fake_lidar_pub_.publish(fake_lidar_msg)

    def timer_callback(self):
        ret, frame = self.cap.read()
        #ret = False
        if ret == True:
            print("Get image")
            img = draw_green_edges(frame)

            self.img_pub_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            # cv2.imshow("image", img)
            #self.occ = get_occ_grid(img)
            min_angle, max_angle, angle_increment, range_min, range_max, ranges = get_occ_grid_alt(img)

            if min_angle is None:
                return
            # min_angle = 0.0
            # max_angle = 0.0
            # angle_increment = 0.0
            # range_min = 0.0
            # range_max = 0.0
            # ranges = np.array([0.0, 1.0])

            #fake_lidar = self.occ2lid()

            fake_lidar_msg = LaserScan()
            fake_lidar_msg.ranges = ranges.tolist()
            fake_lidar_msg.angle_min = min_angle
            fake_lidar_msg.angle_max = max_angle
            fake_lidar_msg.angle_increment = angle_increment
            fake_lidar_msg.range_min = range_min
            fake_lidar_msg.range_max = range_max

            # fake_lidar_msg.header.frame_id = "ego_racecar/base_link"
            fake_lidar_msg.header.frame_id = "laser"
            fake_lidar_msg.header.stamp = self.get_clock().now().to_msg()

            self.fake_lidar_pub_.publish(fake_lidar_msg)


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # TODO Implement this, if even necessary!
        proc_ranges = ranges
        return proc_ranges

    def isInside(self, x, y):
        return abs(x) <= self.occ_size[0] / 2 and abs(y) <= self.occ_size[1] / 2
    
    def is_free(self, x, y):
        x_idx, y_idx = self.map_to_index(x, y)

        # Check if the cell is within the grid bounds
        if x_idx >= 0 and x_idx < self.occ.shape[0] and y_idx >= 0 and y_idx < self.occ.shape[1]:
            # Check if the cell is free
            if self.occ[x_idx, y_idx] == 1:
                return False
        return True
    
    def map_to_index(self, x, y):
        i = int(np.floor((x + self.occ_size[0] / 2) / self.resolution))
        j = int(np.floor((-y + self.occ_size[1] / 2) / self.resolution))
        return i, j

    def publish_occ(self):

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

    def scan_callback(self, scan_msg):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        self.initilize_FL_msg_vars(scan_msg)

        # TODO Replace this stuff once OccGrid generated from Camera, not LiDAR
        self.lid2occ(scan_msg)

        self.publish_occ()

        fake_lidar = self.occ2lid()
        
        self.print_both_lidars(50, scan_msg, fake_lidar)
        
    def print_both_lidars(self, gap_size, scan_msg, fake_lidar):
        
        print("New readings, with gap size", gap_size)
        for i in range (int(len(fake_lidar) / gap_size)):
            # print(scan_msg.ranges[int(gap_size*i)], fake_lidar[int(gap_size*i)])

            print("{:.2f} , {:.2f}".format(scan_msg.ranges[int(gap_size*i)], fake_lidar[int(gap_size*i)]))
    
    def initilize_FL_msg_vars(self, scan_msg):

        self.FL_angle_min = scan_msg.angle_min
        self.FL_angle_max = scan_msg.angle_max
        self.FL_angle_increment = scan_msg.angle_increment
        self.FL_range_min = scan_msg.range_min
        self.FL_range_max = scan_msg.range_max
        self.FL_time_increment = scan_msg.time_increment
        self.FL_scan_time = scan_msg.scan_time
        self.FL_intensities = scan_msg.intensities
    
    def lid2occ(self, scan_msg):

        ranges = scan_msg.ranges
        ranges = self.preprocess_lidar(ranges)

        angle = scan_msg.angle_min
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


    # Input: Occupancy grid, namely a 2D array, simply self.occ
    # Output: LiDAR ranges, which is also published to '/fake_scan' topic
    def occ2lid(self):

        fake_lidar = []
        angle = self.FL_angle_min
        i = 0
        while angle <= self.FL_angle_max:

            (x,y,dis_i) = (0,0,0)
            while self.isInside(x,y):
                if not self.is_free(x, y):
                    break
                dis_i += self.resolution
                x = dis_i * np.cos(angle)
                y = dis_i * np.sin(angle)
            
            if self.isInside(x,y):
                fake_lidar.append(dis_i)
            else:
                fake_lidar.append(self.FL_range_max)
            
            angle += self.FL_angle_increment
            i += 1

        # Publish and return new LiDAR scan array
        # TODO Update these values when testing from camera, not real LiDARs
        # TODO Namely angle_min, angle_max, angle_increment, 
        fake_lidar_msg = LaserScan()
        fake_lidar_msg.ranges = fake_lidar
        fake_lidar_msg.angle_min = self.FL_angle_min
        fake_lidar_msg.angle_max = self.FL_angle_max
        fake_lidar_msg.angle_increment = self.FL_angle_increment
        fake_lidar_msg.range_min = self.FL_range_min
        fake_lidar_msg.range_max = self.FL_range_max
        fake_lidar_msg.time_increment = self.FL_time_increment
        fake_lidar_msg.scan_time = self.FL_scan_time
        fake_lidar_msg.intensities = self.FL_intensities

        fake_lidar_msg.header.frame_id = "ego_racecar/base_link"
        fake_lidar_msg.header.stamp = self.get_clock().now().to_msg()


        self.fake_lidar_pub_.publish(fake_lidar_msg)
        
        return fake_lidar


def main(args=None):
    rclpy.init(args=args)
    print("OccupancyGridToLiDAR Initialized")
    occ2lid_node = OccupancyGridToLiDAR()
    rclpy.spin(occ2lid_node)

    occ2lid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
