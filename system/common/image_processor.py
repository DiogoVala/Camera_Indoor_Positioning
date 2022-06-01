import threading
import cv2
import time
import numpy as np
# Thread management
nProcess = 20 # Number of threads to run
ImgProcessorPool = []

class ImageProcessor(threading.Thread):
	def __init__(self, processor_fcn, w, h):
		super(ImageProcessor, self).__init__()
		self.w = w
		self.h = h
		self.processor_fcn = processor_fcn
		self.event = threading.Event()
		self.terminated = False
		self.frame = None
		self.frameID = None
		self.start()

	def run(self):
		# This method runs in a separate thread
		while not self.terminated:
			# Wait for an image to be written to the stream
			self.event.wait(timeout=None)
			
			if self.frame is not None:					

				self.frame=self.frame.reshape(self.h*3//2,self.w)
				frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_YUV420p2RGB)
				frame_yuv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2YUV)

				self.processor_fcn(self.frameID, frame_yuv) # Call function to process frame
			'''
			try:
				if self.frame is not None:					
					
					self.frame=self.frame.reshape(self.h*3//2,self.w)
					frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_YUV420p2RGB)
					frame_yuv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2YUV)
				
					self.processor_fcn(self.frameID, frame_yuv) # Call function to process frame
			except:
				print("error")
			finally:
				self.event.clear()
				ImgProcessorPool.append(self)
			'''
