import sys
import socket
import time


def client(msg):
    host = '127.0.0.1'
    port = 12345
    # while msg != 'q' and msg != 'exit':
    s = socket.socket()
    s.connect((host, port))
    s.send(str(msg).encode('utf-8'))
    data = s.recv(1024).decode('utf-8')
    print('Received from Server: ', data)
    s.close()
    # return data

while 1:
    with open("/tmp/robopos", 'r') as file1:
        pos = file1.readline()
        client(pos)
    time.sleep(1)