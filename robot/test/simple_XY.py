#!/usr/bin/python

import sys
import socket
from math import pi, cos, sin
from random import random
import unicodedata
import time


def getpoint(h, k, r):
    theta = random() * 2 * pi
    # return h + cos(theta) * r, k + sin(theta) * r
    return (50, 50)


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
    print ap1
    # centre = list(ap1.split(","))
    [x, y] = [0, 0]
    while True:
        x = x + 1
        if x % 2 == 0:
            y = y + 1
        # getpoint(int(float(centre[1])),int(float(centre[2])), 60)
        #msg = 'set.mybot.setPosition("'
        msg = 'set.sta1.setPosition("'
        msg = msg + str(int(x)) + ',' + str(int(y)) + ',0")'
        print msg
        result = client(msg)
        print client("get.sta1.position")
        time.sleep(0.5)
        if x == 300:
            break
