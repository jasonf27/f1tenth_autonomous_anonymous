#!/usr/bin/env python3

import numpy as np
import cv2
import glob
import os 

checkerboard_size = (7, 9)
square_size = 0.025 # meters

#calibration_images = glob.glob('calibration/*.jpg')
data_path = os.path.join('/home/student/final_project/final-project-autonomous-anonymous/calibration','*g') 
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
print(K)