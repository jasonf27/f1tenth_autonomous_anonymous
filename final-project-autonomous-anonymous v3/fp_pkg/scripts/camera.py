#!/usr/bin/env python3
import cv2
import time
import numpy as np
import os


cap = cv2.VideoCapture(4)
cap.set(3, 960)
cap.set(4, 540)

fx, fy = 693.67797621, 694.76838489  # focal lengths
cx, cy = 448.53498348, 258.05020739  # optical center
camera_height = 0.12  # height from the ground
pitch_angle = 0.0  # pitch angle in radians

grid_size = 0.01  # size of each grid cell in meters
grid_width = 2.0  # width of the occupancy grid in meters
grid_height = 2.0  # height of the occupancy grid in meters
angle_increment = 0.01  # angle increment in radians
range_max = 30.0 # maximum range of the laser scanner in meters
range_min = 0.0 # minimum range of the laser scanner in meters

# cap = cv2.VideoCapture("v4l2src device=/dev/video2 extra-controls=\"c,exposure_auto=3\" ! video/x-raw, width=960, height=540 ! videoconvert ! video/x-raw,format=BGR ! appsink")

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

    _, binary_img = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)

    binary_img[:int(binary_img.shape[0]/6), :] = 0


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

def get_occ_grid(img):
    height, width = img.shape
    # Create the occupancy grid (initialize as all free space)
    grid = np.zeros((int(grid_height / grid_size), int(grid_width / grid_size)))

    # Create 2D arrays for pixel coordinates
    u, v = np.meshgrid(np.arange(width), np.arange(height))

    # Identify the occupied pixels
    occupied_pixels = img == 255

    # Calculate the real-world coordinates (X, Y) in the car's frame
    Z = camera_height / np.cos(pitch_angle - np.arctan((v - cy) / fy))  # depth (Z) for each pixel
    X = Z * np.sin(pitch_angle - np.arctan((v - cy) / fy))  # X now corresponds to forward direction (depth in the image)
    Y = -(u - cx) * Z / fx  # Y now corresponds to leftward direction

    print(X.max(), Y.max(), Z.max())

    # Calculate the grid cell coordinates
    grid_x = (X / grid_size).astype(int)
    grid_y = (Y / grid_size).astype(int)

    # Update the occupancy grid
    valid = (0 <= grid_x) & (grid_x < grid.shape[1]) & (0 <= grid_y) & (grid_y < grid.shape[0])
    grid[grid_y[valid & occupied_pixels], grid_x[valid & occupied_pixels]] = 1

    return grid

def get_occ_grid_alt(img):
    height, width = img.shape
    # Create the occupancy grid (initialize as all free space)
    angles = []

    for x in range(width):
        for y in range(height):
            if img[y, x] == 255:  # if the pixel is occupied     
                x_car = fy * (camera_height - 0) / (y - cy)
                y_car = -x_car*(x - cx)/fx

                angle = np.arctan2(y_car, x_car)
                distance = np.sqrt(x_car**2 + y_car**2)
                angles.append([angle, distance])
    
    angles = np.array(angles)

    # Convert the data into a structured numpy array for ease of sorting
    dtype = [('angle', float), ('distance', float)]
    structured_data = np.array(list(zip(angles[:, 0], angles[:, 1])), dtype=dtype)

    # Sort the structured array by angle
    sorted_data = np.sort(structured_data, order='angle')

    # Round the angles to the nearest hundredth
    rounded_angles = np.round(sorted_data['angle'], 2)

    # Find the unique rounded angles and their indices
    unique_angles, indices = np.unique(rounded_angles, return_index=True)

    # For each unique rounded angle, find the smallest distance
    min_distances = np.minimum.reduceat(sorted_data['distance'], indices)

    # Generate an array for all possible angles from min to max with the given increment
    all_angles = np.arange(rounded_angles.min(), rounded_angles.max() + 0.01, 0.01)
    all_angles = np.round(all_angles, 2)  # Ensure that the rounding matches

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
    angle_increment = np.diff(all_angles).mean()  # This assumes the angle increment is roughly constant

    print('Minimum Angle: ', min_angle)
    print('Maximum Angle: ', max_angle)
    print('Angle Increment: ', angle_increment)
    print('Minimum Range: ', np.min(all_distances))
    print('Maximum Range: ', np.max(all_distances))
    print(output.shape)

    return min_angle, max_angle, angle_increment, range_min, range_max, all_distances


# # Example usage:
# image_path = "/home/alan/ESE6150/final-project-autonomous-anonymous/images/nomeasure.png"
# image = cv2.imread(image_path)
# image_result = draw_green_edges(image)
# get_occ_grid_alt(image_result)
# grid = get_occ_grid(image_result)
# norm_grid = (grid / grid.max() * 255).astype(np.uint8)

# # Check if the image is not empty
# if image is None:
#     print(f"Error: Unable to read the image at '{image_path}'. Please check the file path or format.")
# else:
#     # Create a window and display the image
#     cv2.namedWindow('Image')
#     cv2.imshow('Image', image_result)
#     cv2.imshow('Grid', norm_grid)

#     # Set the mouse click event callback
#     # cv2.setMouseCallback('Image', on_mouse_click)

#     # Wait for a key press and close the window
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

image_dir = "/home/nvidia/f1tenth_ws/src/final-project-autonomous-anonymous/images/"
isExist = os.path.exists(image_dir)
if not os.path.exists(image_dir):
    os.makedirs(image_dir)
    print('Creating directory:', image_dir)
window_name = "Camera Stream"
cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("BEV", cv2.WINDOW_AUTOSIZE)

# Define the source points (the trapezoid)
src_points = np.float32([
    [380, 360],
    [579, 360],
    [959, 539],
    [0, 539]
])

i = 0
while True:
    ret, frame = cap.read() # Read a frame from the video capture device

    if ret:
        param = {
            'img': frame
        }
        cv2.setMouseCallback(window_name, on_mouse_click, param)
        image_result = draw_green_edges(frame)
        bev_image = homography(frame, src_points, 300, 900)
        cv2.imshow(window_name, frame) # Display the RGB frame in a window
        cv2.imshow("BEV", image_result)
        if cv2.waitKey(1) == ord('q'): # Press q to quit
            break
        elif cv2.waitKey(1) == ord('s'):
            print("Save Image")
            cv2.imwrite(image_dir + str(i) + ".png", frame)
            cv2.imwrite(image_dir + str(i) + "bev.png", bev_image)
            i += 1

cap.release() # Release the video capture device
cv2.destroyAllWindows() # Close all windows
