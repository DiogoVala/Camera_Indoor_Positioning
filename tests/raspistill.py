import csv
import subprocess as sp
import atexit


videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --nopreview -ex sports -ISO 150"
cameraProcess = sp.atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly
(videoCmd, stdout=sp.PIPE, bufsize=1)
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly


atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly
