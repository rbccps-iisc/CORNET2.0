import os
import rospy
import math
from visualization_msgs.msg import Marker

import sys
import socket



# Connect the socket to the port where the server is listening
server_address = '/tmp/uds_socket'





robots = ['/charlie', '/snoopy', '/spike']
wifi_stations = ['sta1', 'sta2', 'sta3']
ns_msg = ["set.", "station", ".setPosition", "(", "\"", "'", "xPosition", ",", "yPosition", ",", "zPosition", "\"", ")",
          "'"]


def mininet_client(msg):
    host = '127.0.0.1'
    port = 12345
    # while msg != 'q' and msg != 'exit':
    s = socket.socket()
    s.connect((host, port))
    s.send(str(msg).encode('utf-8'))
    data = s.recv(1024).decode('utf-8')
    #print('Received from Server: ', data)
    s.close()
    return data

def get_pose(msg):
    recv_robot = msg.ns
    # print(msg.ns,msg.pose.position)
    # print(msg.pose.position)
    # print(robots.index(recv_robot))
    # index = robots.index(recv_robot)
    ns_msg[1] = wifi_stations[robots.index(recv_robot)]
    ns_msg[6] = msg.pose.position.x
    ns_msg[8] = msg.pose.position.y
    ns_msg[10] = msg.pose.position.z
    try:
        # Create a UDS socket
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        #print('connecting to %s' % server_address)
        sock.connect(server_address)
        print(''.join([str(item) for item in ns_msg]))
        sock.sendall(''.join([str(item) for item in ns_msg]).encode('utf-8'))
        sock.close()
        #sock.sendall(":SEND\n".encode('utf-8'))
        #print(mininet_client(''.join([str(item) for item in ns_msg])))
        rospy.sleep(1)
    except socket.error as msg:
        print(msg)
        #sys.exit(1)
    finally:
        print('closing socket')
        #close(sock)
        #sock.close()

def robot_position_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/visualization_marker", Marker, get_pose)
        # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    robot_position_listener()
