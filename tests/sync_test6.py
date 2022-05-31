import cv2, queue as Queue, threading, time
import picamera
from picamera.array import PiYUVArray

camera_resolution = (2028, 1520)
Terminated = False

class VideoCaptureQ:

	def __init__(self, name):
		self.camera = picamera.PiCamera()
		self.camera.resolution = camera_resolution
		self.camera.framerate = 10
		self.camera.exposure_mode = 'sports'
		self.camera.sensor_mode = 2

		self.rawCapture = PiYUVArray(self.camera, size=camera_resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture, format="yuv", use_video_port=True)
		
		self.q = Queue.Queue()
		t = threading.Thread(target=self._reader)
		t.daemon = True
		t.start()

	# read frames as soon as they are available, keeping only most recent one
	def _reader(self):
		for f in self.stream:
			frame = f.array
			self.rawCapture.truncate(0)
			# if the thread indicator variable is set, stop the thread
			# and resource camera resources
			if not self.q.empty():
				try:
					self.q.get_nowait()   # discard previous (unprocessed) frame
				except Queue.Empty:
					pass
			self.q.put(frame)
			
			if Terminated:
				break

		#On thread exit
		self.camera.close()

	def read(self):
		return self.q.get()

import time

cap = VideoCaptureQ(0)

while True:

	t1 = time.time()
	try:
		ori_frame = cap.read()
		#print(ori_frame.shape)
		#cv2.imshow("frame", ori_frame)
		#key=cv2.waitKey(33)
		#if key == ord('q'):
			#break

		# do your stuff
	except Exception as e:
		print(e)
		break
	t2 = time.time()
	print(f'FPS: {1/(t2-t1)}')

Terminated = True
