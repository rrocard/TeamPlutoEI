import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch the joyteleop node
    joyteleop_node = Node(
        package='lower_level',
        executable='joyteleop',  # Ensure this matches the correct executable name
        output='screen',
    )
    ld.add_action(joyteleop_node)

    # Launch the speed_controller node
    speed_controller_node = Node(
        package='lower_level',
        executable='speed_controller',  # Ensure this matches the correct executable name
        output='screen',
    )
    ld.add_action(speed_controller_node)

    return ld
