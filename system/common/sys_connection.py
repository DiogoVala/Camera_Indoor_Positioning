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
        self.event = threading.Event()
        self.connected = False
        self.start()
        while not self.connected:
            pass
	
    def run(self):
        try:    
            while True:
                while not self.connected:
                    try:
                        print("Initiating socket server.")
                        self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.s.bind((HOST, PORT))
                        self.rxdata = None
                        print("Waiting for client connection.")
                        self.s.listen()
                        self.conn, self.addr = self.s.accept()
                        print(f"Connected by client {self.addr}")
                        self.connected = True
                        atexit.register(self.clean)
                    except Exception as e:
                        print(e)
                        time.sleep(5)
                    
                
                while self.connected:
                    self.rxdata = self.conn.recv(1024)
                    self.rxdata = self.rxdata.decode('utf-8')
                    if not self.rxdata:
                        print("Terminating socket server")
                        self.connected = False
                    else:
                        #print("Received:", eval(self.rxdata))
                        try:
                            self.rxdata = eval(self.rxdata)
                            heapq.heappush(self.dataQ, self.rxdata)
                        except:
                            pass
                        #self.output_fcn(self.rxdata)
                        self.event.set() # Set event signal on data acquisition
        except:   
            self.s.close()
    def clean(self):
        self.s.send("\0")
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
        while not self.connected:
            pass
        
    def run(self):
        while True:
            while not self.connected:
                try:
                    self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.s.connect((HOST, PORT))
                    self.connected = True
                except Exception as e:
                    self.connecting_retries += 1
                    print(e)
                    if self.connecting_retries == Socket_Con_Retries:
                        print(f"Could not find server at {HOST}.")
                        break
                    print("Retrying connection.")
                    time.sleep(1)
            
            while self.connected:
                if self.event.wait():
                    try:
                        message=str.encode(str(self.txdata))
                        print("Sending:", message)
                        self.s.send(message)
                    except Exception as e:
                        print(e)
                        self.s.close()
                        self.connected = False
                    finally:
                        self.event.clear()
            try:
                s.close()
            except:
                pass
