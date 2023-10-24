#!/usr/bin/env python3
import cv2
import time
import numpy as np
import os

# Mouse click event callback function
def on_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Get the BGR value of the clicked point
        image = param['img']
        bgr_value = image[y, x]
        
        # Convert the BGR value to HSV
        hsv_value = cv2.cvtColor(np.uint8([[bgr_value]]), cv2.COLOR_BGR2HSV)[0][0]
        
        print(f"Clicked at ({x}, {y}) - HSV value: {hsv_value}")
    
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

    return blurred

def homography(image, src_points, width, height):
    # Define the destination points (the rectangle)
    dst_points = np.float32([
        [0, 0],
        [width-1, 0],
        [width-1, height-1],
        [0, height-1]
    ])

    # Compute the perspective transformation matrix
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Apply the perspective transformation to the image
    warped_image = cv2.warpPerspective(image, matrix, (width, height))

    # cv2.imshow('Original Image', image)
    # cv2.imshow('Bird\'s Eye View', warped_image)

    return warped_image

image_path = "/home/alan/ESE6150/final-project-autonomous-anonymous/images/checkerboard.png"
image = cv2.imread(image_path)
image_result = draw_green_edges(image)

# Define the source points (the trapezoid)
src_points = np.float32([
    [312, 360],
    [647, 360],
    [959, 539],
    [0, 539]
])
new_image = homography(image, src_points, 600, 400)
if image is None:
    print(f"Error: Unable to read the image at '{image_path}'. Please check the file path or format.")
else:
    # window_name = "Image"
    # cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('Image', image)
    # param = {
    #     'img': image
    # }
    # cv2.setMouseCallback(window_name, on_mouse_click, param)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
