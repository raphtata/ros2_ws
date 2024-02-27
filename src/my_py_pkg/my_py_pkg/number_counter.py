#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from rclpy.qos import ReliabilityPolicy, QoSProfile
from example_interfaces.srv import SetBool

class numberCounter(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('robot_news_station')
        self.timer_period = 1.0
        ##subscriber##
        self.subscriber = self.create_subscription(Int64,'number',
        self.listener_callback,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)) 
    
        ##publisher##
        self.publisher_ = self.create_publisher(Int64, 'number_count', 10)
        self.counter = 0

        ##server##
        self.server = self.create_service(
            SetBool, "counter_resize" , self.callback_setBool)
    
    def callback_setBool(self , request, response):

        response.success = request.data
        if request.data : 
            self.counter = 0
        return response
    
    def call_setBool_client(self, data):

        client = self.create_client(SetBool, "counter_resize" )
            
        while not client.wait_for_service(1.0):
            self.get_logger().info( "wait for server")
        
        request = SetBool.Request()
        request.data = data
         
        #get the result 
        future = client.call_async(request)
        future.add_done_callback(self.call_back_result)

    def call_back_result(self, future):
        try : 
            result = future.result()
            if result: 
                self.counter = 0
        except Exception as e:
            self.get_logger().info("service call failed %r" %(e,))

    
    def listener_callback(self,msg):

        self.counter = self.counter + msg.data

        self.get_logger().info("subscriber msg : " + str(self.counter) + " " + str(msg.data))
        if msg.data == 55 :
            self.call_setBool_client(True) ##remap to 0
        msg_send = Int64()
        msg_send.data = self.counter
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    mynode = numberCounter()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()