#!/usr/bin/env python3

import cv2
import numpy as np

def detect_yellow_lanes(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the yellow color in HSV space
    lower_yellow = np.array([20, 53, 150])
    upper_yellow = np.array([36, 255, 255])

    # Create a mask to extract only the yellow pixels in the image
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Apply a Gaussian blur to the masked image
    blur = cv2.GaussianBlur(mask, (5, 5), 0)

    # Apply Canny edge detection to the blurred image
    edges = cv2.Canny(blur, 50, 150)

    # Create a mask to extract only the region of interest
    mask = np.zeros_like(edges)
    height, width = edges.shape
    vertices = np.array([[(0, height), (width/2, height/2), (width, height)]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    rho = 3
    theta = np.pi / 180
    threshold = 15
    min_line_len = 150
    max_line_gap = 60
    lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    
    # Draw the detected lines on the original image
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # Combine the line image with the original image
    result = cv2.addWeighted(image, 0.8, line_image, 1, 0)

    return result

# Load an image
image = cv2.imread('image.jpg')

# Detect yellow lanes in the image
result = detect_yellow_lanes()

# Display the result
cv2.imshow('Result', result)
cv2.waitKey(0)
cv2.destroyAllWindows()
