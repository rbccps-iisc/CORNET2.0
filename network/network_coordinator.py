#!/usr/bin/python

import multiprocessing
import os
import socket
import struct
import sys

import yaml
import threading


class CornetNM:
    def __init__(self, config):
        # event used to indicate whether threads are allowed to run
        self.run_event = multiprocessing.Event()

        with open(config) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        self.models = self.config['gazebo_models']
        self.node_type = self.config['type']
        self.pose = config['pose']
        self.ip_list = config['ip_list']

    def _network_setup(self):
        pass

def recv_one_message(cls, sock):
    lengthbuf = cls.recvall(sock, 4)
    if not lengthbuf: return None
    length, = struct.unpack('!I', lengthbuf)
    return cls.recvall(sock, length)

def send_one_message(sock, data):
    length = len(data)
    sock.sendall(struct.pack('!I', length) + data)

def network_recv_stub(node):
    server_address = '/tmp/' + node +'recv'
    print server_address
    # Make sure the socket does not already exist
    try:
        os.unlink(server_address)
    except OSError:
        if os.path.exists(server_address):
            raise
    # Create a UDS socket
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(server_address)

    try:
        try:
            sock.listen(1)
            connection, client_address = sock.accept()
            while True:
                data = recv_one_message(connection)
                print data
                #FIXTIT do something with data

        except socket.error:
            raise KeyboardInterrupt
    except KeyboardInterrupt:
        print("exiting the process")
    finally:
        sock.close()
        os.unlink(server_address)

def network_send_stub(node):
    server_address = '/tmp/' + node +'send'
    print server_address
    # Make sure the socket does not already exist
    try:
        os.unlink(server_address)
    except OSError:
        if os.path.exists(server_address):
            raise
    # Create a UDS socket
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(server_address)

    try:
        try:
            sock.listen(1)
            connection, client_address = sock.accept()
            while True:
                #data = send_one_message(connection)
                #print data
                #FIXTIT do something with data
                pass

        except socket.error:
            raise KeyboardInterrupt
    except KeyboardInterrupt:
        print("exiting the process")
    finally:
        sock.close()
        os.unlink(server_address)


def main(args):
    if len(args) != 2:
        print("usage: network_coordinator.py <config_file>")
    else:
        # creating thread
        t1 = threading.Thread(target=network_recv_stub, args=(args[1],))
        t2 = threading.Thread(target=network_send_stub, args=(args[1],))

        # starting thread 1
        t1.start()
        # starting thread 2
        t2.start()

        # wait until thread 1 is completely executed
        t1.join()
        # wait until thread 2 is completely executed
        t2.join()

        # both threads completely executed
        print("Done!")
        #network_recv_stub(args[1])


if __name__ == '__main__':
    sys.exit(main(sys.argv))