from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    spawner_node = Node(
        package = "turtle_project",
        executable = "turtle_spawner_node",
        parameters=[
            {"spawner_frequency" : 1.0}
        ]
    )

    controller_node = Node(
        package = "turtle_project",
        executable = "turtle_controller_node"
    )

    turtle_node = Node(
        package = "turtlesim",
        executable = "turtlesim_node"
    )
    
    ld.add_action(turtle_node)
    ld.add_action(spawner_node)
    ld.add_action(controller_node)

    return ld