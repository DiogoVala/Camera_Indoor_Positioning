import csv
import subprocess as sp
import atexit


videoCmd = "raspistill -o ~/Camera_Indoor_Positioning/tests/cal.jpg -w 2016 -h 1520"
sp.call(videoCmd, shell=True)


