import threading
from picamera.array import PiYUVArray
import numpy
import time

# Thread management
nProcess = 20 # Number of threads to run
ImgProcessorLock = threading.Lock() # Interprocess variable for mutual exclusion
ImgProcessorDone = False # Global to indicate end of processing (To stop threads)
ImgProcessorPool = []

class ImageProcessor(threading.Thread):
	def __init__(self, processor_fcn, camera, resolution):
		super(ImageProcessor, self).__init__()
		self.processor_fcn = processor_fcn
		self.stream = PiYUVArray(camera, size=resolution)
		self.event = threading.Event()
		self.terminated = False
		self.start()

	def run(self):
		# This method runs in a separate thread
		while not self.terminated:
			# Wait for an image to be written to the stream
			if self.event.wait(1):
				try:
					#print(f"\n{threading.current_thread()} at: {datetime.datetime.now()}")
					self.stream.seek(0)
					frame = self.stream.array
					if frame is not None:
						print(time.time())
						self.processor_fcn(frame) # Call function to process frame
				finally:
					self.stream.seek(0)
					self.stream.truncate()
					self.event.clear()
					#with ImgProcessorLock:
					ImgProcessorPool.append(self)


# Generator of buffers for the capture_sequence method.
# Each buffer belongs to an ImageProcessor so each frame is sent to a different thread.
def getStream():
	while not ImgProcessorDone:
		with ImgProcessorLock:
			if ImgProcessorPool:
				processor = ImgProcessorPool.pop()
				yield processor.stream
				processor.event.set()
			else:
				#time.sleep(0.1)
				print("ImgProcessorPool empty")
				#break
