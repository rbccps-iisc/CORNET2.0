#!/usr/bin/env python
"""update position from ros robot to network simulator(mininet/containernet)"""

import socket
import sys
import time

import rclpy
from gazebo_msgs.srv import GetEntityState
from rclpy.node import Node


def client(msg):
    host = '127.0.0.1'
    port = 12345
    # while msg != 'q' and msg != 'exit':
    s = socket.socket()
    s.connect((host, port))
    s.send(str(msg).encode('utf-8'))
    data = s.recv(1024).decode('utf-8')
    print('Received from Server: ', data)
    s.close()
    return data

class RclpyClient(Node):
    def __init__(self):
        super().__init__('minimal_client_async')

        self.cli = self.create_client(GetEntityState, '/cornet/get_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = GetEntityState.Request()

    def send_request(self, model_name):
        self.request.name = model_name
        self.response = self.cli.call_async(self.request)

    def client(self, msg):
        host = '127.0.0.1'
        port = 12345
        # while msg != 'q' and msg != 'exit':
        s = socket.socket()
        s.connect((host, port))
        s.send(str(msg).encode('utf-8'))
        data = s.recv(1024).decode('utf-8')
        print('Received from Server: ', data)
        s.close()
        return data


if __name__ == '__main__':
    # rclpy.init(args=sys.argv)
    # node = rclpy.create_node('position_update')
    #
    # node.get_logger().info('Created node')
    # cli = node.create_client(GetEntityState, '/demo/get_entity_state')
    #
    # req = GetEntityState.Request()
    # node.get_logger().debug(req.get_fields_and_field_types())
    # req.name = "name_robo0"
    # req.reference_frame = ""
    # while not cli.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().debug('service not available, waiting again...')
    #
    # while True:
    #     try:
    #         resp = cli.call_async(req)
    #         rclpy.spin_until_future_complete(node, resp)
    #         result = resp.result()
    #         node.get_logger().debug(result)
    #         print (result.state.pose.position.x, result.state.pose.position.y)
    #         #FIXME sleep
    #         time.sleep(1)
    #     except KeyboardInterrupt:
    #         break
    robots = ['robot0', 'robot1', 'robot2', 'robot3']
    while True:
        try:
            for robot in robots:
                rclpy.init(args=sys.argv)
                R1 = RclpyClient()
                R1.send_request(str(robot))
                rclpy.spin_once(R1)
                if R1.response.done():
                    result = R1.response.result()
                    print (result.state.pose.position.x, result.state.pose.position.y)
                    x = result.state.pose.position.x
                    y = result.state.pose.position.y
                    #print(x,y)
                    msg = 'set.'+ robot +'.setPosition("' + str(int(x)) + ',' + str(int(y)) + ',0")'
                    result = R1.client(msg)
                #R1.destroy_node()
                rclpy.shutdown()
                time.sleep(0.4)


        except KeyboardInterrupt:
            break


