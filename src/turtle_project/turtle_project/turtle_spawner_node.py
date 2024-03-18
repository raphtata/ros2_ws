#! /usr/bin/env python3

import rclpy
import random
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.msg import TurtleInfo
from my_robot_interfaces.msg import TurtleAlive
from my_robot_interfaces.srv import CatchTurtle

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

class TurtleSpawner(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('turtle_spawn_node')
        self.timer_period = 1.0
        self.turtle_count_ = 0
        self.timer_pawner= self.create_timer(self.timer_period, self.timer_spawner_callback)
        self.server = self.create_service(
            CatchTurtle, "catch_service" , self.turtle_service_callback)
        self.turtle_list_ = []
        self.turtle_alive_publisher_ = self.create_publisher(TurtleAlive, 'alive_turtles', 10)
        self.timer = self.create_timer(self.timer_period, self.turtles_update)
    
    def turtles_update(self): 
        msg = TurtleAlive()
        msg.turtles = self.turtle_list_ 
        self.turtle_alive_publisher_.publish(msg)

    def turtle_service_callback(self, request, response):
        turtle_to_catch = request.name
        #delete name in the list
        self.turtle_list_  = [turtle for turtle in self.turtle_list_ if turtle.name != turtle_to_catch ]
        #kill turtle node 
        self.kill_turtle(request.name)
        self.turtles_update()
        #for turtle in self.turtle_list_: 
        #    if turtle.name == turtle_to_catch :
        #        del self.turtle_list_.remove(turtle)
        response.success = True
        return response

    def kill_turtle(self, name): 
        client = self.create_client(Kill, "kill" )
            
        while not client.wait_for_service(1.0):
            self.get_logger().info( "wait for server")
        
        request = Kill.Request()
        request.name = name

        #get the result 
        future = client.call_async(request)
        future.add_done_callback(partial(self.kill_turtle_node_callback, name = request.name))

    def kill_turtle_node_callback(self,future, name) :
        try : 
            result = future.result()
            if result : 
                self.get_logger().info("Turtle " + str(name) + " has been deleted from node list " )
        except Exception as e:
            self.get_logger().info("service call failed %r" %(e,))

    def timer_spawner_callback(self):

        self.turtle_count_ += 1 
        client = self.create_client(Spawn, "spawn" )
            
        while not client.wait_for_service(1.0):
            self.get_logger().info( "wait for server")
        
        request = Spawn.Request()
        request.x = random.uniform(0.0, 10.0)
        request.y = random.uniform(0.0, 10.0) 
        request.theta = random.uniform(0.0, 3.4) 
        request.name = "coco" + str(self.turtle_count_)
        turtle = TurtleInfo()
        turtle.name = request.name 
        turtle.x = request.x 
        turtle.y = request.y 
        turtle.theta = request.theta 

        self.turtle_list_.append(turtle)
        #get the result 
        future = client.call_async(request)
        future.add_done_callback(partial(self.call_back_result, name = request.name))
    
    def call_back_result(self, future, name):
        try : 
            result = future.result()
            if result : 
                self.get_logger().info("Turtle " + str(name) + " has been created " )
        except Exception as e:
            self.get_logger().info("service call failed %r" %(e,))


def main(args=None):

    rclpy.init(args=args)
    mynode = TurtleSpawner()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()