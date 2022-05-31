import threading
import numpy
import time
import cv2

# Thread management
nProcess = 100 # Number of threads to run
ImgProcessorLock = threading.Lock() # Interprocess variable for mutual exclusion
ImgProcessorDone = False # Global to indicate end of processing (To stop threads)
ImgProcessorPool = []

class ImageProcessor(threading.Thread):
	def __init__(self, processor_fcn):
		super(ImageProcessor, self).__init__()
		self.processor_fcn = processor_fcn
		self.event = threading.Event()
		self.terminated = False
		self.frame = None
		self.start()

	def run(self):
		# This method runs in a separate thread
		while not self.terminated:
			# Wait for an image to be written to the stream
			if self.event.wait(1):
				try:
					if self.frame is not None:
						self.frame=self.frame.reshape(1520*3//2,2016)
						print("newframe")
						frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_YUV420p2RGB)
						frame_yuv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2YUV)
						self.processor_fcn(frame_yuv) # Call function to process frame
				finally:
					self.event.clear()
					ImgProcessorPool.append(self)

