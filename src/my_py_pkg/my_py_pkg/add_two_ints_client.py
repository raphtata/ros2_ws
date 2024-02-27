#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('add_two_ints_client')
        self.call_add_two_ints_server(3, 7)
        self.call_add_two_ints_server(8, 8)


        
    def call_add_two_ints_server(self, a, b):

        client = self.create_client(AddTwoInts, "add_two_ints" )
            
        while not client.wait_for_service(1.0):
            self.get_logger().info( "wait for server")
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b 
         
        #get the result 
        future = client.call_async(request)
        future.add_done_callback(partial(self.call_back_result, a = a, b = b))

    def call_back_result(self, future, a, b):
        try : 
            result = future.result()
            self.get_logger().info("resultat : " +str(a) + " + " + str(b) + " = " + str(result.sum))
        except Exception as e:
            self.get_logger().info("service call failed %r" %(e,))


def main(args=None):

    rclpy.init(args=args)
    mynode = AddTwoIntsClientNode()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()