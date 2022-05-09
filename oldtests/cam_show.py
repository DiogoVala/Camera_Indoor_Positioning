import cv2
import numpy as np
import time

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
time.sleep(2)

while True:
    ret,frame = cam.read()
    frame = cv2.flip(frame, 0)
    cv2.imshow('webcam', frame)
    
	
    if cv2.waitKey(1)&0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
