#! /usr/bin/env python3

import rclpy
import math 
from rclpy.node import Node
from turtlesim.msg import Pose 
from functools import partial
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleInfo
from my_robot_interfaces.msg import TurtleAlive
from my_robot_interfaces.srv import CatchTurtle

class TurtleController(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('turtle_controller_node')
        self.pose_ = None

        self.turtle_to_catch_ = None
        self.distPtTr_ = 0.0
        ##subscriber##
        self.subscriber = self.create_subscription(Pose,'/turtle1/pose',
        self.listener_callback,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)) 
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(TurtleAlive, 'alive_turtles', self.turtle_alive_callback, 10)
        self.control_loop_timer = self.create_timer(0.01, self.control_callback)
    
    def distance(self, x2, x1 , y2 , y1):

        disX = x2 - x1
        disY = y2 - y1
        return math.sqrt(disX*disX + disY*disY)
  
    def angular (self , x2, x1 , y2 , y1):

        self.distPtTr_ = self.distance(x2, x1 , y2 , y1)
        alpha = math.atan2 (y2 - y1 , x2 - x1)
        return alpha
    
    def catch_closer_turtle(self, turtles):
        min_dist = math.inf
        turtle_to_catch = None
        for  turlte in turtles : 
            if self.distance(turlte.x, self.pose_.x,turlte.y, self.pose_.y ) < min_dist : 
                min_dist = self.distance(turlte.x, self.pose_.x,turlte.y, self.pose_.y )
                turtle_to_catch = turlte
        return turtle_to_catch
    
    def turtle_alive_callback(self,msg):
        if len(msg.turtles) > 0:
            self.turtle_to_catch_ = self.catch_closer_turtle(msg.turtles)

    def control_callback(self):

        if self.pose_ == None or self.turtle_to_catch_ == None: 
            return 
                
        self.get_logger().info("subscriber msg y : " + str(self.pose_.y) + " x :" + str(self.pose_.x))

        msg_to_send = Twist()
            
        angle_dest = self.angular(self.turtle_to_catch_.x, self.pose_.x , self.turtle_to_catch_.y , self.pose_.y)
        self.get_logger().info("send angular cmd : " + str(angle_dest))
        angular_diff = angle_dest - self.pose_.theta

        if angular_diff > math.pi :
            angular_diff -= 2*math.pi
        elif angular_diff < -math.pi :
            angular_diff += 2*math.pi

        #angle
        msg_to_send.angular.z = 6*angular_diff

        #position
        msg_to_send.linear.x  = 2*self.distPtTr_ 
        
        if self.distPtTr_ < 0.4 :
            msg_to_send.linear.x  = 0.0
            msg_to_send.angular.z = 0.0
            self.get_logger().info("stop to run")
            self.turtle_to_remove(self.turtle_to_catch_.name)
        
        self.publisher_.publish(msg_to_send)
    
    def turtle_to_remove(self, name):

        client = self.create_client(CatchTurtle, "catch_service")
            
        while not client.wait_for_service(1.0):
            self.get_logger().info( "wait for server")
        
        request = CatchTurtle.Request()
        request.name = name

        #get the result 
        future = client.call_async(request)
        future.add_done_callback(partial(self.call_back_result, name = request.name))
        
    def call_back_result(self, future, name):
        try : 
            result = future.result()
            if result : 
                self.get_logger().info("Turtle " + str(name) + " has been deleted " )
        except Exception as e:
            self.get_logger().info("service call failed %r" %(e,))

    def listener_callback(self,msg):
        self.pose_ = msg

        


def main(args=None):

    rclpy.init(args=args)
    mynode = TurtleController()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()