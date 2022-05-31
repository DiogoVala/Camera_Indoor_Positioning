# import the necessary packages
from picamera.array import PiRGBArray
from picamera.array import PiYUVArray
from picamera import PiCamera
from threading import Thread
import cv2
import sys
from imutils.video import FPS
import time

RESOLUTION=(680, 420)

class PiVideoStream:
	def __init__(self, resolution=RESOLUTION, framerate=30):
		# initialize the camera and stream
		print("hello")
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.framerate = framerate
		self.rawCapture = PiRGBArray(self.camera, size=resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)
		self.frame = None
		self.stopped = False
		self.frameID = 0
		
	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self
		
	def update(self):
		# keep looping infinitely until the thread is stopped
		for f in self.stream:
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame
			self.frameID += 1
			sys.stdout.flush()
			self.frame = f.array
			self.rawCapture.truncate(0)
			# if the thread indicator variable is set, stop the thread
			# and resource camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.camera.close()
				return
				
	def read(self):
		# return the frame most recently read
		return self.frameID, self.frame
		
	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from `picamera` module...")
vs = PiVideoStream().start()
time.sleep(0.1)
fps = FPS().start()

# loop over some frames...this time using the threaded stream
oldFrameID = -1
newFrameID = 0
while fps._numFrames <= 50:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	newFrameID, frame = vs.read()
	
	if newFrameID != oldFrameID:
		oldFrameID = newFrameID
		fps.update()
	
	if frame is not None:
		cv2.imshow("Frame", frame)
	key=cv2.waitKey(33)
	if key == ord('q'):
		break;
	
# stop the timer and display FPS information

fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
