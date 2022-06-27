#!/usr/bin/python

import sys
import socket
from math import pi, cos, sin
from random import random
import unicodedata
import time

def client(msg):
    host = '127.0.0.1'
    port = 12345
    while msg != 'q' and msg != 'exit':
        s = socket.socket()
        s.connect((host, port))
        s.send(str(msg).encode('utf-8'))
        data = s.recv(1024).decode('utf-8')
        print('Received from Server: ', data)
        return data
        s.close()

if __name__ == '__main__':
    # get the postion of AP1
    ap1 = client("get.ap1.position")
    ap1 = unicodedata.normalize('NFKD', ap1).encode('ascii', 'ignore')
    ap1 = ap1.replace("[", " ")
    ap1 = ap1.replace("]", " ")
    print( ap1)
    # centre = list(ap1.split(","))
    args = sys.argv

    if len(args) != 2:
        # print("usage: network_config.py <config_file>")
        # print args[1]
        [x, y] = [1, 0]

    else:
        print( args[1], args[2])
        [x, y] = [args[1], float(args[2])]


