#!/usr/bin/env python3
import cv2
import numpy as np

# Mouse click event callback function
def on_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Get the BGR value of the clicked point
        bgr_value = image[y, x]
        
        # Convert the BGR value to HSV
        hsv_value = cv2.cvtColor(np.uint8([[bgr_value]]), cv2.COLOR_BGR2HSV)[0][0]
        
        print(f"Clicked at ({x}, {y}) - HSV value: {hsv_value}")

def draw_green_edges(image_path, area_threshold=20):
    # Read the image
    image = cv2.imread(image_path)

    # Check if the image is not empty
    if image is None:
        print(f"Error: Unable to read the image at '{image_path}'. Please check the file path or format.")
        return

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the yellow color in HSV space
    lower_yellow = np.array([20, 53, 150])
    upper_yellow = np.array([36, 255, 255])

    # Create a mask for yellow color
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Apply the mask to the original image
    masked_image = cv2.bitwise_and(image, image, mask=mask)

    # Convert the masked image to grayscale
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to remove noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detect edges using Canny edge detector
    edges = cv2.Canny(blurred, 50, 200)

    # Dilate the edges to close gaps in the detected lines
    kernel = np.ones((2, 2), np.uint8)
    dilated_edges = cv2.dilate(edges, kernel, iterations=1)

    # cv2.imshow('Green Edges', dilated_edges)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Find contours of the dilated edges
    contours, _ = cv2.findContours(dilated_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    filtered_contours = [contour for contour in contours if cv2.contourArea(contour) >= area_threshold]


    # Draw green contours on the original image
    green_edges_image = cv2.drawContours(image.copy(), filtered_contours, -1, (0, 255, 0), 2)

    # Save the result
    output_image_path = "green_edges.png"
    cv2.imwrite(output_image_path, green_edges_image)

    # Show the result
    cv2.imshow('Green Edges', green_edges_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return green_edges_image

# Example usage:
image_path = 'resource/lane.png'
image_result = draw_green_edges(image_path)

cv2.imwrite(image_result, "lane_result.png")

# image = cv2.imread(image_path)

# # Check if the image is not empty
# if image is None:
#     print(f"Error: Unable to read the image at '{image_path}'. Please check the file path or format.")
# else:
#     # Create a window and display the image
#     cv2.namedWindow('Image')
#     cv2.imshow('Image', image)

#     # Set the mouse click event callback
#     cv2.setMouseCallback('Image', on_mouse_click)

#     # Wait for a key press and close the window
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()