#!/usr/bin/env python3

import cv2
import time
from convert_trt import detect
from PIL import Image

def pixel_to_world_coordinates(img_x, img_y):
    
    # mounting_height = 0.12755257702117701
    mounting_height = 0.01
    fx = 693.67797621
    fy = 694.76838489
    u0 = 448.53498348
    v0 = 258.05020739

    x_car = fy *(mounting_height)/ (img_y - v0)
    y_car = -x_car*(img_x - u0)/fx

    print("Distance in x_car coordinate:", x_car)
    print("Distance in y_car coordinate:", y_car)

    return x_car, y_car


cap = cv2.VideoCapture(4)

# cap = cv2.VideoCapture("v4l2src device=/dev/video2 extra-controls=\"c,exposure_auto=3\" ! video/x-raw, width=960, height=540 ! videoconvert ! video/x-raw,format=BGR ! appsink")

counter = 0
average_window = 100

while True:
    ret, frame = cap.read() # Read a frame from the video capture device
    window_name = "Camera Stream"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    if ret:
        img_pil = Image.fromarray(frame)
        #passing to the model
        u, v = detect(onnx_file_path="lab8model.onnx", trt_file_path="lab8model32.trt", input_img=img_pil, cv2window=window_name)

        if u == -1 or v == -1:
            print("Error: got no pixels")
        else:
            x_car, y_car = pixel_to_world_coordinates(u, v)
            if counter == 0:
                start = time.time()
            counter += 1

        if counter == average_window:
            end = time.time()
            average_time = (end - start) / (average_window - 1)
            print("Average inference time: ", average_time)
            exit(0)

        key = cv2.waitKey(1)
        if key == ord('q'):
            exit(0)

