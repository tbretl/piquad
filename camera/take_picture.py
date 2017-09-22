import picamera

# Create an instance of the PiCamera class
camera = picamera.PiCamera()

# Take a picture and save it
camera.capture('test.jpg')
