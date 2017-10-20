from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

camera = PiCamera()
rawCapture = PiRGBArray(camera)

time.sleep(0.1)

camera.capture(rawCapture, format="bgr")
image = rawCapture.array

image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
cv2.imwrite('test.pgm', image)
