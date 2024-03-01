from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_publisher_node = Node(
        package = "my_py_pkg",
        executable = "number_publisher",
        parameters=[
            {"number_to_publish" : 5}
        ]
    )

    number_counter_node = Node(
        package = "my_py_pkg",
        executable = "number_counter"
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)

    return ld