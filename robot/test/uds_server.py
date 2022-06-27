import socket

import socket
import sys
import os

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


while True:
    # Wait for a connection
    print >> sys.stderr, 'waiting for a connection'
    connection, client_address = sock.accept()
    try:
        data = connection.recv(1024)
        if data:
            print(data)
            print(mininet_client(data))
    finally:
        # Clean up the connection
        connection.close()
