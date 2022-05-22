from picamera import PiCamera
import time

camera = PiCamera()
time.sleep(2)
camera.resolution = (2016, 1520)
camera.framerate = 30

file_name = "video_" + str(time.time()) + ".h264"

print("Start recording...")
camera.start_recording(file_name)
camera.wait_recording(5)
camera.stop_recording()
camera.close()
print("Done.")
