#!/usr/bin/env python
"""update position from ros robot to network simulator(mininet)"""

import rospy
import math
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState

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
    return data


def ros_position_update():
    #rospy.init_node('check_odometry')
    relative_entity_name = ''
    model_name = 'mybot'
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        return gms(model_name, relative_entity_name)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    # rospy.spin()


if __name__ == '__main__':
    while True:
        resp = ros_position_update()
        print resp
        x = resp.pose.position.x
        y = resp.pose.position.y
        msg = 'set.mybot.setPosition("' + str(int(x)) + ',' + str(int(y)) + ',0")'
        result = client(msg)
        print result
        rospy.sleep(1)
