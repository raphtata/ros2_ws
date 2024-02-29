#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from rclpy.qos import ReliabilityPolicy, QoSProfile

class RobotNewsStation(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('robot_news_station')
        self.declare_parameter("robot_name" , "C3PO")

        self.robot_name_ = self.get_parameter("robot_name" ).value
        self.timer_period = 1.0
        ##publisher##
        self.publisher_ = self.create_publisher(String, 'robot_news', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.counter = 0
       
    def timer_callback(self):
        msg = String()
        self.counter +=1
        msg.data = "Hi, I am " + str(self.robot_name_) + " robot to serve"
        self.publisher_.publish(msg)

def main(args=None):

    rclpy.init(args=args)
    mynode = RobotNewsStation()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()