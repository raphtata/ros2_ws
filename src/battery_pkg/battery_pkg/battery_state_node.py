#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.srv import BatteryState

class BatteryStateNode(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('battery_state_node')
        self.call_led_server( 3, True)
        self.timer_off_period = 4.0
        self.timer_on_period = 6.0

        self.timer_off = self.create_timer(self.timer_off_period, self.timer_off_callback)
        self.timer_on  = self.create_timer(self.timer_on_period, self.timer_on_callback)

    
    def call_led_server(self, led, state):

        client = self.create_client(BatteryState, "battery_state" )
            
        while not client.wait_for_service(1.0):
            self.get_logger().info( "wait for server")
        
        request = BatteryState.Request()
        request.led_number = led
        request.state = state 
         
        #get the result 
        future = client.call_async(request)
        future.add_done_callback(partial(self.call_back_result, led = led, state = state))
    
    def timer_off_callback(self):
        self.call_led_server( 1, 0)
        #at the end of the timer off , it resets the timer on
        self.timer_off.cancel()
        self.timer_on.reset() 

    def timer_on_callback(self):
        self.call_led_server( 1, 1)
        self.timer_on.cancel()
        #at the end of the timer on , it resets the timer off
        self.timer_off.reset()

    def call_back_result(self, future, led, state):
        try : 
            result = future.result()
            if result : 
                self.get_logger().info("Led " + str(led) + "has changed and is " + str("on" if state == 1 else "off"))
        except Exception as e:
            self.get_logger().info("service call failed %r" %(e,))


def main(args=None):

    rclpy.init(args=args)
    mynode = BatteryStateNode()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()