import socket
import numpy as np
import pickle
import time

def client_program():
    host = "192.168.82.10"  # as both code is running on same pc
    port = 8080  # socket server port number

    client_socket = socket.socket()  # instantiate
    client_socket.connect((host, port))  # connect to the server

    message = np.array([123, [1,2,3]],dtype=object)

    while True:
        client_socket.send(pickle.dumps(message))  # send message
        time.sleep(0.1)

    client_socket.close()  # close the connection


if __name__ == '__main__':
    client_program()
