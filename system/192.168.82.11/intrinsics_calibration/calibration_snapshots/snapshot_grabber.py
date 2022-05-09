import cv2
import time
import sys
import os
import picamera
from picamera.array import PiRGBArray
import numpy as np

# Settings
#(4032, 3040)
FRAME_WIDTH = 4032
FRAME_HEIGHT = 3040
SAVE_FOLDER = "./new_snapshots_%dx%d" % (FRAME_WIDTH, FRAME_HEIGHT)
FILE_NAME = "snapshot"

def save_snaps(width, height, name, folder):

    try:
        if not os.path.exists(folder):
            os.makedirs(folder)
            folder = os.path.dirname(folder)
            try:
                os.stat(folder)
            except:
                os.mkdir(folder)
    except:
        pass


    with picamera.PiCamera() as camera:
        print (f"Saving snapshots with resolution {width}x{height}.")
        print ("Press SPACE to capture.")
        
        camera.resolution=(width, height)
        capture = PiRGBArray(camera, size=(width, height))
            
        nSnap=0
        fileName    = "%s/%s_%d_%d_" %(folder, name, width, height)
        while True:
    
            camera.capture(capture, 'rgb')
            frame = capture.array
            capture.truncate(0)
            
            frame_g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            frame_resized = cv2.resize(frame_g, (int(width/3), int(height/3)))
            cv2.imshow('Snapshot Preview', frame_resized)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord(' '):
                print("Saving image ", nSnap)
                cv2.imwrite("%s%d.jpg"%(fileName, nSnap), frame_g)
                nSnap += 1

    cv2.destroyAllWindows()

def main():
    save_snaps(width=FRAME_WIDTH, height=FRAME_HEIGHT, name=FILE_NAME, folder=SAVE_FOLDER)

    print("Files saved.")

if __name__ == "__main__":
    main()
