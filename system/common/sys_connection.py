import socket
import threading
import time
import atexit
import heapq

HOST = "192.168.82.10"   # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

Socket_Con_Retries = -1 # Infinite retries

class Socket_Server(threading.Thread):
    def __init__(self, output_fcn, dataQ):
        super(Socket_Server, self).__init__()
        self.output_fcn = output_fcn
        self.dataQ = dataQ
        print("Initiating socket server.")
        self.terminated = False
        self.event = threading.Event()
        self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected = False
        while not self.connected:
            try:
                self.s.bind((HOST, PORT))
                self.connected = True
            except:
                print("Socket address is already in use. Retrying in a few seconds.")
                time.sleep(5)
        atexit.register(self.s.close)
        self.rxdata = None
        print("Waiting for client connection.")
        self.s.listen()
        self.conn, self.addr = self.s.accept()
        print(f"Connected by client {self.addr}")
        self.start()
	
    def run(self):
        with self.conn:
            #print("Server connection valid.")
            while not self.terminated:
                self.rxdata = self.conn.recv(1024)
                self.rxdata = self.rxdata.decode('utf-8')
                if not self.rxdata:
                    print("Terminating socket server")
                    self.terminated = True
                else:
                    #print("Received:", eval(self.rxdata))
                    try:
                        self.rxdata = eval(self.rxdata)
                        heapq.heappush(self.dataQ, self.rxdata)
                    except:
                        pass
                    #self.output_fcn(self.rxdata)
                    self.event.set() # Set event signal on data acquisition
            self.s.close()
	    
class Socket_Client(threading.Thread):
    def __init__(self):
        super(Socket_Client, self).__init__()
        self.terminated = False
        self.connected = False
        self.connecting_retries = 0
        self.event = threading.Event()
        self.txdata=None
        self.start()
    def run(self):
        while True:
            while not self.connected:
                try:
                    self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.s.connect((HOST, PORT))
                    self.connected = True
                    self.terminated = False
                except:
                    self.connecting_retries += 1
                    print("Connection failed.")
                    if self.connecting_retries == Socket_Con_Retries:
                        self.terminated = True
                        print(f"Could not find server at {HOST}.")
                        break
                    print("Retrying connection.")
                    time.sleep(1)
            
            while not self.terminated:
                if self.event.wait():
                    try:
                        message=str.encode(str(self.txdata))
                        print("Sending:", message)
                        self.s.send(message)
                    except:
                        print("Could not send data to server.")
                        self.s.close()
                        self.terminated = True
                        self.connected = False
                    finally:
                        self.event.clear()
            try:
                s.close()
            except:
                pass
