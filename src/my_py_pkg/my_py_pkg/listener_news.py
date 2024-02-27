#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ListenerNewsStation(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('listener_news')
        self.counter = 0
        ##subscriber##
        self.subscriber = self.create_subscription(String,'robot_news',
        self.listener_callback,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.

    def listener_callback(self,msg):
        self.counter += 1
        self.get_logger().info("subscriber msg : " + str(self.counter) + " " + str(msg.data))


def main(args=None):

    rclpy.init(args=args)
    mynode = ListenerNewsStation()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()