import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch the joyteleop node once
    joyteleop_node = Node(
        package='lower_level',             
        executable='joyteleop',            
        output='screen',                   
        name='joyteleop'                   
    )
    ld.add_action(joyteleop_node)

    # Launch the speed_controller node
    speed_controller_node = Node(
        package='lower_level',
        executable='speed_controller',
        output='screen',
        name='speed_controller'
    )
    ld.add_action(speed_controller_node)

    return ld
