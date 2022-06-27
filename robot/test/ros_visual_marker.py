import os
import rospy
import math
from visualization_msgs.msg import Marker

import sys
import socket



# Connect the socket to the port where the server is listening
server_address = '/tmp/uds_socket'

# Create a UDS socket
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

print ('connecting to %s' % server_address)
sock.connect(server_address)

robots = ['/charlie', '/snoopy', '/spike']
wifi_stations = ['sta1', 'sta2', 'sta3']
ns_msg = ["set.", "station", ".setPosition", "(", "\"", "'", "xPosition", ",", "yPosition", ",", "zPosition", "\"", ")",
          "'"]


def get_pose(msg):
    recv_robot = msg.ns
    # print(msg.ns,msg.pose.position)
    # print(msg.pose.position)
    # print(robots.index(recv_robot))
    # index = robots.index(recv_robot)
    ns_msg[1] = wifi_stations[robots.index(recv_robot)]
    ns_msg[6] = msg.pose.position.x  # .decode('utf-8')
    ns_msg[8] = msg.pose.position.y
    ns_msg[10] = msg.pose.position.z

    try:
        #sock.connect(server_address)
        # print(''.join([str(item) for item in ns_msg]))
        sock.sendall(''.join([str(item) for item in ns_msg]))
        # print(client(''.join([str(item) for item in ns_msg])))
    except socket.error as msg:
        print( msg)
        #sys.exit(1)
    finally:
        print ('closing socket')
        #close(sock)
        #sock.close()

def robot_position_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/visualization_marker", Marker, get_pose)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    robot_position_listener()
