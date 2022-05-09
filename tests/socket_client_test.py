from sys_connection import *
import time
import threading

socket_clt = Socket_Client()

N=100

while N:
	socket_clt.txdata=(1,2,3)
	socket_clt.event.set()
	time.sleep(1)
	N-=1
socket_clt.join()
