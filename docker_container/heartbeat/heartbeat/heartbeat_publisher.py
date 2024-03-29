#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import Header

import sys

class HeartbeatPublisher(Node):
    def __init__(self, args):
        super().__init__('heartbeat_publisher')

        self.declare_parameter('heartbeat_period_s', 0.5)
        if len(args) != 2:
            print("usage: network_config.py robotNode")
        else:
            self.robot = args[1]
            heartbeat_period_s = self.get_parameter('heartbeat_period_s').value
            self.get_logger().info(
                'Starting heartbeat with period {}'.format(heartbeat_period_s)
            )
            self._pub = self.create_publisher(Header, 'heartbeat', 1)
            self._clock = Clock()
            self._timer = self.create_timer(heartbeat_period_s, self.timer_callback)

    def timer_callback(self):
        timestamp = self._clock.now()
        msg = Header()
        msg.stamp = timestamp.to_msg()
        msg.frame_id = self.robot
        self._pub.publish(msg)
        self.get_logger().debug('Publishing: "%s"' % msg.stamp)


def main(args=None):
    rclpy.init(args=args)

    publisher = HeartbeatPublisher(sys.argv)

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
