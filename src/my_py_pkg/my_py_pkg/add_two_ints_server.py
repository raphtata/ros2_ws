#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('add_two_ints_server')

        self.get_logger().info("add_two_ints_server node started")

        self.server = self.create_service(
            AddTwoInts, "add_two_ints" , self.callback_add_two_ints)

       
    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info("response.sum : " + str(response.sum))
        return response

def main(args=None):

    rclpy.init(args=args)
    mynode = AddTwoIntsServerNode()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()