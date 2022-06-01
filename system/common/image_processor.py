import threading
import cv2
import time
import numpy as np
# Thread management
nProcess = 10 # Number of threads to run
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
			try:
				if self.frame is not None:					
					
					start=time.time()
					self.frame=self.frame.reshape(self.h*3//2,self.w)
					frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_YUV420p2RGB)
					frame_yuv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2YUV)
					#print("opencv",time.time()-start)
					'''
					start=time.time()
					y = self.frame[0:self.h*self.w].reshape((self.h, self.w))  # Read Y color channel and reshape to height x width numpy array
					u = self.frame[self.h*self.w:self.h*self.w+self.h//2*self.w//2].reshape((self.h//2, self.w//2))  # Read U color channel and reshape to height x width numpy array
					v = self.frame[self.h*self.w+self.h//2*self.w//2:self.h*self.w+self.h//2*self.w//2+2*self.h//2*self.w//2].reshape((self.h//2, self.w//2))  # Read V color channel and reshape to height x width numpy array
					u = cv2.resize(u, (self.w, self.h))
					v = cv2.resize(v, (self.w, self.h))
					frame_yuv = np.dstack((y,u,v))
					print("yuv",time.time()-start)
					'''
					
					self.processor_fcn(self.frameID, frame_yuv) # Call function to process frame
			finally:
				self.event.clear()
				ImgProcessorPool.append(self)

