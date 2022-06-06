import csv
import subprocess as sp
import atexit


<<<<<<< HEAD
videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --nopreview -ex sports -ISO 150"
cameraProcess = sp.atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly
(videoCmd, stdout=sp.PIPE, bufsize=1)
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly
=======
videoCmd = "raspistill -o ~/Camera_Indoor_Positioning/tests/cal.jpg -w 2016 -h 1520"
sp.call(videoCmd, shell=True)
>>>>>>> 23ea3b333d4d7e2488868c67c3500fcf1dcfa885

