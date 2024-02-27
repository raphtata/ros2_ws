#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

class Mynode(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('py_test')
        self.main_task_period = 1.0
        self.counter = 0
        self.create_timer(self.main_task_period, self.main_task)


    def main_task(self):
        self.counter+=1
        self.get_logger().info("hello Ros2 !!!4!" + str(self.counter))

def main(args=None):

    rclpy.init(args=args)
    mynode = Mynode()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()