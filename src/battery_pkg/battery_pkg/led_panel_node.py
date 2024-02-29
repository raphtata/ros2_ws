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

        #declare led array as a parameter
        self.declare_parameter("led_states", [0, 0, 0])
        self.led_states_ = self.get_parameter("led_states").value
        self.server = self.create_service(
            BatteryState, "battery_state" , self.callback_led_panel)

        self.led_publisher_ = self.create_publisher(LedPanel, 'led_panel_state', 10)
    
    def publish_led_states(self):
        msg = LedPanel()
        msg.led = self.led_states_
        self.led_publisher_.publish(msg)

    def callback_led_panel(self, request, response):
        led_number = request.led_number
        state = request.state

        if led_number > len(self.led_states_) or led_number <= 0:
            response.success = False
            return response

        if state not in [0, 1]:
            response.success = False
            return response

        self.led_states_[led_number - 1] = state
        response.success = True
        self.publish_led_states()
        return response

def main(args=None):

    rclpy.init(args=args)
    mynode = LedPanelNode()
    rclpy.spin(mynode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()