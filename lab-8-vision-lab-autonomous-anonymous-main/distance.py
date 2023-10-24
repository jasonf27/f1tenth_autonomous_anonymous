#!/usr/bin/env python3

import numpy as np
import cv2
import glob
import os 

checkerboard_size = (6, 8)
square_size = 0.25 # meters

#calibration_images = glob.glob('calibration/*.jpg')
data_path = os.path.join('/home/student/lab8_ws/lab-8-vision-lab-autonomous-anonymous/calibration','*g') 
calibration_images = glob.glob(data_path) 

object_points = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
object_points[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2) * square_size

image_points = []

for image_file in calibration_images:
    image = cv2.imread(image_file)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    if ret:
        image_points.append(corners)

ret, K, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera([object_points] * len(image_points), image_points, gray.shape[::-1], None, None)

print("Camera Intrinsic Matrix:")
print(K), gray.shape[::-1], None, None

img = cv2.imread('resource/cone_x40cm.png')

# hsv color space as per the lectures
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# color ranges for the red cone, change these values based on detection estimates
lower_red = (0, 140, 100)
upper_red = (10, 255, 255)

# creating a mask
mask = cv2.inRange(hsv, lower_red, upper_red)

# finding contours
contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Iterating through the contours to find the largest one
largest_contour = None
largest_area = 0
for contour in contours:
    area = cv2.contourArea(contour)
    if area > largest_area:
        largest_contour = contour
        largest_area = area

# bounding box of the largest contour
x, y, w, h = cv2.boundingRect(largest_contour)

# calculating the pixel coordinates of the lower right corner of the bounding box
lower_right_x = x + w 
lower_right_y = y + h


img_x = lower_right_x - 480
img_y = lower_right_y - 270
print("Pixel coordinates of lower right corner: ({}, {})".format(img_x, img_y))

img_x, img_y = (184, 228)
x_car = 40 # cm
fx = K[0,0]
fy = K[1,1]
u0 = K[0,2]
v0 = K[1,2]

mounting_height = ((img_y)*x_car - v0)/fy
print(mounting_height)
mounting_height = mounting_height/100

img = cv2.imread('resource/cone_unknown.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# color ranges for the red cone, change these values based on detection estimates
lower_red = (0, 140, 100)
upper_red = (10, 255, 255)

# creating a mask
mask = cv2.inRange(hsv, lower_red, upper_red)

# finding contours
contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# iterating through the contours to find the largest one
largest_contour = None
largest_area = 0
for contour in contours:
    area = cv2.contourArea(contour)
    if area > largest_area:
        largest_contour = contour
        largest_area = area

# bounding box of the largest contour
x, y, w, h = cv2.boundingRect(largest_contour)

# calculating the pixel coordinates of the lower right corner of the bounding box
lower_right_x = x + w 
lower_right_y = y + h

img_x = lower_right_x
img_y = lower_right_y

mounting_height = 0.12755257702117701
fx = 693.67797621 
fy = 694.76838489
u0 = 448.53498348
v0 = 258.05020739

x_car = fy *(mounting_height)/ (img_y - v0)
y_car = -x_car*(img_x - u0)/fx

print("Distance in x_car coordinate:", x_car)
print("Distance in y_car coordinate:", y_car) 



def calib_mounting_height(img_y, x_car):
    mounting_height = ((img_y)*x_car - v0)/fy
    print(mounting_height)
    mounting_height = mounting_height/100

    return mounting_height



def pixel_to_world_coordinates(img_x, img_y):
    mounting_height = 0.12755257702117701
    fx = 693.67797621 
    fy = 694.76838489
    u0 = 448.53498348
    v0 = 258.05020739

    x_car = fy *(mounting_height)/ (img_y - v0)
    y_car = -x_car*(img_x - u0)/fx

    print("Distance in x_car coordinate:", x_car)
    print("Distance in y_car coordinate:", y_car) 

    return x_car, y_car
