#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisher(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('number_publisher')
        self.declare_parameter("number_to_publish", 2)

        self.number_ = self.get_parameter("number_to_publish").value
        self.timer_period = 1.0
        ##publisher##
        self.publisher_ = self.create_publisher(Int64, 'number', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.counter = 0
       
    def timer_callback(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    mynode = NumberPublisher()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()