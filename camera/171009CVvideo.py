# 171009 openCV video practice

import numpy as np
import cv2
import picamera
from picamera.array import PiRGBArray
import time

#set up the camera
camera = picamera.PiCamera()
camera.vflip = True #vertical flip
camera.video_stabilization = True #stabilize the camera
camera.framerate = 32
camera.resolution = (640,480)
rawCapture = PiRGBArray(camera, size=(640,480))

#allow camera to warm up
time.sleep(0.1)

#Capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #grab the raw NumPy array representing the image, then initizae the timestamp
    #and occupied/unoccupied text
    image = frame.array

    #show the frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF

    #clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    #if the 'q' key was pressed, break from the loop
    if key == ord("q"):
        break
