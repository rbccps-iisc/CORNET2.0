#!/usr/bin/env python
"""
    update position from ros robot to network simulator(mininet)
    using Unix Domain Sockets establishes the datapath
"""
import struct

import rospy
import math
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState
import yaml
import socket
import os


class CornetRM:
    def __init__(self, config_file):
        self.gen_config = {}

        with open(config_file) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        self.host = '127.0.0.1'
        self.port = 12345

    def mn_wifi_client(self, msg):
        # while msg != 'q' and msg != 'exit':
        s = socket.socket()
        s.connect((self.host, self.port))
        s.send(str(msg).encode('utf-8'))
        data = s.recv(1024).decode('utf-8')
        print('Received from Server: ', data)
        s.close()
        return data

    def gazebo_pos_update(self):
        relative_entity_name = ''
        models = self.config['gazebo_models']
        node_type = self.config['type']
        for idx, model_name in enumerate(models):
            rospy.wait_for_service('/gazebo/get_model_state')
            try:
                if node_type[idx] == "MOBILE":
                    gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                    resp = gms(model_name, relative_entity_name)
                    x = resp.pose.position.x
                    y = resp.pose.position.y
                    msg = 'set.'+model_name+'.setPosition("' + str(int(x)) + ',' + str(int(y)) + ',0")'


            except rospy.ServiceException, e:
                print "Service call failed: %s" % e


