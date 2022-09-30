import socket
import pickle
import numpy as np

def server_program():
    # get the hostname
    host = "192.168.82.10" 
    port = 8080  # initiate port no above 1024

    server_socket = socket.socket()  # get instance
    # look closely. The bind() function takes tuple as argument
    server_socket.bind((host, port))  # bind host address and port together

    # configure how many client the server can listen simultaneously
    server_socket.listen(2)
    conn, address = server_socket.accept()  # accept new connection
    print("Connection from: " + str(address))
    while True:
        # receive data stream. it won't accept data packet greater than 1024 bytes
        data = conn.recv(1024)
        if not data:
            # if data is not received break
            break
        data = pickle.loads(data)
        print(np.array(data,dtype=object))

    conn.close()  # close the connection


if __name__ == '__main__':
    server_program()
