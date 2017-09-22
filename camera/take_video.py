import picamera
from time import sleep

# Create an instance of the PiCamera class
camera = picamera.PiCamera()

# Start recording
camera.start_recording('test.h264')

# Keep recording for 5 seconds
sleep(5)

# Stop recording and save
camera.stop_recording()

