import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description(): 

    ld = LaunchDescription()

    # Include Twist2Pic node (assuming it's in the same workspace)
    twist2pic_node = Node(
        package='twist2pic',
        executable='twist2pic',
        output='screen',
        parameters=[
            {'input_topic': '/bebop/cmd_vel'}  # Make sure this matches your joy_teleop output topic
        ]
    )
    ld.add_action(twist2pic_node)

    # Include JoyTeleop node
    joy_teleop_node = Node(
        package='joy_teleop', 
        executable='joy_teleop',
        output='screen'
    )
    ld.add_action(joy_teleop_node)

    # Include Image_view node
    image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='image_view',
        remappings=[
            ('image', '/image_out/compressed')
        ]
    )
    ld.add_action(image_view_node)

    return ld