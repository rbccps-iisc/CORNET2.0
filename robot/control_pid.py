#!/usr/bin/env python
""" LIP control """
import os
import socket
import struct

import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, WrenchStamped
import numpy as np


def recv_one_message(cls, sock):
    lengthbuf = cls.recvall(sock, 4)
    if not lengthbuf: return None
    length, = struct.unpack('!I', lengthbuf)
    return cls.recvall(sock, length)


def send_one_message(sock, data):
    length = len(data)
    sock.sendall(struct.pack('!I', length) + data)


class LIPController(object):
    """ LIP Control """

    def __init__(self, timestep):
        self.velocity = 0
        self.effort = 0
        self.position = 0
        self.kp = 150
        self.ki = 10
        self.kd = 150
        self.target_theta = 0
        self.smp_time = timestep
        self.theta0 = 0
        self.thetai = 0
        self.error = 0
        self.duty_ratio = 0
        self.pubs = rospy.Publisher("/lip/joint_effort_controller_j_0/command/", Float64, queue_size=10)
        self.command_sub = rospy.Subscriber("/lip/joint_states", JointState, self.callback)

    def callback(self, data):
        """ Callback function for subscribing /lip/joint_states topic
        Use this data in your control equation below """

        server_address = '/tmp/liprecv'
        src = {"src": "lip",
               "dest": "mybot",
               "data": data,
               "time": rospy.get_rostime()
               }
        try:
            os.unlink(server_address)
        except OSError:
            if os.path.exists(server_address):
                raise
        # Create a UDS socket
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.bind(server_address)
        try:
            send_one_message(sock, data)

            self.effort = data.effort
            self.velocity = data.velocity
            self.position = data.position
            """ Write your control equation in self.cmd 
            to publish the effort values to the Joint Effort Controller """
            self.error = self.target_theta - self.position[0]
            self.dtheta = (self.error - self.theta0) / self.smp_time
            self.theta0 = self.error
            self.thetai += self.error * self.smp_time
            self.duty_ratio = (self.error * self.kp) + (self.thetai * self.ki) + (self.dtheta * self.kd)
            self.cmd = self.duty_ratio
            if self.cmd > 100:
                self.cmd = 100
            elif self.cmd < -100:
                self.cmd = -100
            self.pubs.publish(self.cmd)
        except:
            print "except"


def main():
    rospy.init_node('lip_control', anonymous=False)
    controller = LIPController(.0001)
    rospy.spin()
    del controller
    return


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
