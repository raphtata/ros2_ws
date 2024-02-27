#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import BatteryState
from my_robot_interfaces.msg import LedPanel

class LedPanelNode(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('led_panel_node')

        self.get_logger().info("led pannel node started")

        self.server = self.create_service(
            BatteryState, "battery_state" , self.callback_led_panel)

        self.led_publisher_ = self.create_publisher(LedPanel, 'led_panel_state', 10)

    def callback_led_panel(self, request, response):
        response.success = True
        self.get_logger().info("Led n: " + str(request.led) + " has changed to " + str("On" if request.state else "Off") )
        
        #publish leds state
        msg = LedPanel()

        if request.state :
            msg.led1 = 0 
            msg.led2 = 0 
            msg.led3 = 1 
            self.led_publisher_.publish(msg)  
        
        else: 
            msg.led1 = 0 
            msg.led2 = 0 
            msg.led3 = 0 
            self.led_publisher_.publish(msg)  

        return response

def main(args=None):

    rclpy.init(args=args)
    mynode = LedPanelNode()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()