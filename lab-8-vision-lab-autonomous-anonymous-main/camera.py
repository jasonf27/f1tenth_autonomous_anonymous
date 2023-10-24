#!/usr/bin/env python3
import cv2
import time

cap = cv2.VideoCapture(4)

# cap = cv2.VideoCapture("v4l2src device=/dev/video2 extra-controls=\"c,exposure_auto=3\" ! video/x-raw, width=960, height=540 ! videoconvert ! video/x-raw,format=BGR ! appsink")

while True:
    ret, frame = cap.read() # Read a frame from the video capture device
    print(frame.shape)

    if ret:
        cv2.imshow('RGB frame', frame) # Display the RGB frame in a window
        if cv2.waitKey(1) == ord('q'): # Press q to quit
            break

cap.release() # Release the video capture device
cv2.destroyAllWindows() # Close all windows
