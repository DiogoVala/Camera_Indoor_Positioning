# Camera-based Positioning System - Diogo Vala 2022

import threading
import cv2

# Thread management
nProcess = 10 # Number of threads to run
ImgProcessorPool = []

class ImageProcessor(threading.Thread):
	def __init__(self, processor_fcn):
		super(ImageProcessor, self).__init__()
		self.processor_fcn = processor_fcn
		self.event = threading.Event()
		self.terminated = False
		self.frame = None
		self.frameID = None
		self.start()

	def run(self):
		while True:
			self.event.wait(timeout=None)
			if self.terminated:
				break
			try:
				if self.frame is not None:				
					self.processor_fcn(self.frameID, self.frame) # Call function to process frame
			except Exception as e:
				#print(e)
				pass
			finally:
				self.event.clear()
				ImgProcessorPool.append(self)
			
