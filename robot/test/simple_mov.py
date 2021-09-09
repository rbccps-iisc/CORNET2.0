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
    return 50, 50


def mov_left(x, y):
    new = x - 0.5
    return new, y


def mov_right(x, y):
    new = x + 0.5
    return new, y


def mov_up(x, y):
    new = y + 0.5
    return x, new


def mov_down(x, y):
    new = y - 0.5
    return x, new


def client(msg):
    host = '127.0.0.1'
    port = 12345
    #while msg != 'q' and msg != 'exit':
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
    args = sys.argv

    if len(args) != 2:
        # print("usage: network_config.py <config_file>")
        #print args[1]
        [x, y] = [1, 0]

    else:
        print args[1]
        [x, y] = [1, float(args[1])]

    while True:
        # x = x + 1
        # if x % 2 == 0:
        #    y = y + 1
        # y = y + 0.5
        # getpoint(int(float(centre[1])),int(float(centre[2])), 60)
        if x <= 1 and y >= 0 and y <= 58:
            x, y = mov_up(x, y)
        elif x >= 1 and x <= 58 and y >= 58:
            x, y == mov_right(x, y)
        elif x >= 58 and y <= 58 and y >= 1:
            x, y == mov_down(x, y)
        elif x <= 58 and x > 1 and y <= 1:
            x, y == mov_left(x, y)
        # msg = 'set.mybot.setPosition("'
        msg = 'set.robot1.setPosition("'
        msg = msg + str(float(x)) + ',' + str(float(y)) + ',0")'
        print msg
        result = client(msg)
        print client("get.robot1.position")

        # print client("get.ap1.position")
        time.sleep(1.0)
        if y == 90:
            break
