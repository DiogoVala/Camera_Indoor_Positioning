import csv
import subprocess as sp
import atexit


videoCmd = "raspistill -o ~/Camera_Indoor_Positioning/tests/cal.jpg -w 2016 -h 1520"
cameraProcess = sp.call(videoCmd, shell=True)
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly

