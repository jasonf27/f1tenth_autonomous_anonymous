#!/usr/bin/env python3
import cv2
import time
import numpy as np
import os

cap = cv2.VideoCapture(4)
cap.set(3, 960)
cap.set(4, 540)

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

    kernel = np.ones((2, 2), np.uint8)
    dilated = cv2.dilate(blurred, kernel, iterations=1)

    return blurred

def on_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Get the BGR value of the clicked point
        image = param['img']
        bgr_value = image[y, x]

        hsv_value = cv2.cvtColor(np.uint8([[bgr_value]]), cv2.COLOR_BGR2HSV)[0][0]
        
        print(f"Clicked at ({x}, {y}) - HSV value: {hsv_value}")

        fx = 693.67797621
        fy = 694.76838489
        u0 = 448.53498348
        v0 = 258.05020739
        # img_y = y
        # x_actual = 0.498475

        # mounting_height = ((img_y- v0)*x_actual)/fy
        # print("mounting height: ", mounting_height)
        # mounting_height = mounting_height/100
        mounting_height = 0.40386
        
        # x_car = fy *(mounting_height - 0)/ (y - v0)
        # y_car = -x_car*(x - u0)/fx

        # print("Distance in x_car coordinate:", x_car)
        # print("Distance in y_car coordinate:", y_car) 

        
        pitch_angle = np.pi / 2 + 0.55
        yaw_angle = 0.08
        # Calculate the real-world coordinates (X, Y) in the car's frame
        angle = pitch_angle + np.arctan((y - v0) / fy)
        print(angle)
        Z = mounting_height / -np.cos(angle)  # depth (Z) for each pixel
        X = Z * np.sin(angle)  # X now corresponds to forward direction (depth in the image)
        Y = -(x - u0) * Z / fx  # Y now corresponds to leftward direction

        # Rotation matrix for the yaw angle
        R_yaw = np.array([[np.cos(yaw_angle), -np.sin(yaw_angle)],
                         [np.sin(yaw_angle), np.cos(yaw_angle)]])

        # Rotate the X_cam and Y_cam coordinates
        X_car, Y_car = np.dot(R_yaw, np.array([X, Y]))

        print("X = 0.38862")
        print("Y = 0.08636")

        print("Distance in X coordinate:", X_car)
        print("Distance in Y coordinate:", Y_car)
        print("Distance in Z coordinate:", Z)


# img = cv2.imread('/home/student/final_project/final-project-autonomous-anonymous/images/measure.png')
img = cv2.imread('/home/alan/ESE6150/final-project-autonomous-anonymous/images/2.png')
image_result = draw_green_edges(img)

window_name = "Camera Stream"
cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

while True:
    param = {
            'img': img
        }
    image_result = draw_green_edges(img)
    cv2.imshow(window_name, img) # Display the RGB frame in a window

    cv2.setMouseCallback(window_name, on_mouse_click, param)

    if cv2.waitKey(1) == ord('q'): # Press q to quit
        break