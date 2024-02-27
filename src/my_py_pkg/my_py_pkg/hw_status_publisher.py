#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('robot_news_station')
        self.timer_period = 1.0
        ##publisher##
        self.hw_status_publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.counter = 0
       
    def timer_callback(self):
        msg = HardwareStatus()
        msg.temperature = 20
        msg.are_motors_ready = True
        msg.debug_message = "Nothing"

        self.hw_status_publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    mynode = HardwareStatusPublisherNode()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()