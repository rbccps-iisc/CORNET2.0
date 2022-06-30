import socket

import socket
import sys
import os
import time
from threading import Thread as thread

server_address = '/tmp/uds_socket'

# Make sure the socket does not already exist
try:
    os.unlink(server_address)
except OSError:
    if os.path.exists(server_address):
        raise

# Create a UDS socket
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

# Bind the socket to the port
print('starting up on %s' % server_address)
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)


def mininet_client(msg):
    host = '127.0.0.1'
    port = 12345
    # while msg != 'q' and msg != 'exit':
    s = socket.socket()
    s.connect((host, port))
    s.send(str(msg).encode('utf-8'))
    data = s.recv(1024).decode('utf-8')
    print('Received from Server: ', data)
    s.close()
    return data


def get_socket_data(conn, addr):
    while True:
        try:
            data = conn.recv(1024).decode('utf-8')
            if data:
                print('received "%s"' % data)
                print(mininet_client(data.strip()))
            else:
                print('no more data from', addr)

            break
        except socket.error as msg:
            print(msg)
            break


while True:
    # Wait for a connection
    print('waiting for a connection')
    connection, client_address = sock.accept()
    # connection.setblocking(0)
    try:
        thread(target=get_socket_data, args=(connection, client_address)).start()
        # while True:
        #     data = connection.recv(1024).decode('utf-8')
        #     if data:
        #         print('received "%s"' % data)
        #         print(mininet_client(data.strip()))


    except socket.error as msg:
        print(msg)
        # sys.exit(1)
    finally:
        # Clean up the connection
        connection.close()
