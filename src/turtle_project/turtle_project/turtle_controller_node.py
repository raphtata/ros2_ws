#! /usr/bin/env python3

import rclpy
import math 
from rclpy.node import Node
from example_interfaces.msg import String
from turtlesim.msg import Pose 
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist


class TurtleController(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('turtle_controller_node')
        self.pose_ = None
        self.x_target = 8.0 
        self.y_target = 4.0
        self.distPtTr_ = 0.0
        ##subscriber##
        self.subscriber = self.create_subscription(Pose,'/turtle1/pose',
        self.listener_callback,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.control_loop_timer = self.create_timer(0.01, self.control_callback)
    
    def angular (self , x2, x1 , y2 , y1):
        disX = x2 - x1
        disY = y2 - y1
        self.distPtTr_ = math.sqrt(disX*disX + disY*disY)

        alpha = math.atan2 (disY , disX)
        return alpha
    
    def control_callback(self):

        if self.pose_ == None : 
            return 
                
        self.get_logger().info("subscriber msg y : " + str(self.pose_.y) + " x :" + str(self.pose_.x))

        msg_to_send = Twist()
            
        angle_dest = self.angular(self.x_target, self.pose_.x , self.y_target, self.pose_.y)
        self.get_logger().info("send angular cmd : " + str(angle_dest))
        angular_diff = angle_dest - self.pose_.theta

        if angular_diff > math.pi :
            angular_diff -= 2*math.pi
        elif angular_diff < -math.pi :
            angular_diff += 2*math.pi

        #angle
        msg_to_send.angular.z = angular_diff

        #position
        msg_to_send.linear.x  = self.distPtTr_ 
        
        if self.distPtTr_ < 0.4 :
            msg_to_send.linear.x  = 0.0
            msg_to_send.angular.z = 0.0
            self.get_logger().info("stop to run")
        
        self.publisher_.publish(msg_to_send)


    def listener_callback(self,msg):
        self.pose_ = msg

        


def main(args=None):

    rclpy.init(args=args)
    mynode = TurtleController()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()