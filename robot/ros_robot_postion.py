#!/usr/bin/env python
"""update postion from ros robot to network simulator(mininet)"""

import rospy
import math
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates

import unicodedata
import sys
import socket


def client(msg):
    host = '127.0.0.1'
    port = 12345
    #while msg != 'q' and msg != 'exit':
    s = socket.socket()
    s.connect((host, port))
    s.send(str(msg).encode('utf-8'))
    data = s.recv(1024).decode('utf-8')
    print('Received from Server: ', data)
    s.close()
    #return data



def callback(msg):
    # instances = len(msg.name)
    x = 0
    y = 0
    print (msg.pose[2].position)
    for idx, val in enumerate(msg.name):
        if val == "mybot":
            # print(msg.pose[idx].position)
            x = msg.pose[idx].position.x
            y = msg.pose[idx].position.y
            print (x,y)
    print(x,y,"asd")

    msg = 'set.sta1.setPosition(' + str(int(x)) + "," + str(int(y)) + ",0)"
    print msg
    result = client(msg)
    #print client("get.sta1.position")




def ros_position_update():
    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.spin()


if __name__ == '__main__':
    ros_position_update()
