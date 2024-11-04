import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory   


def generate_launch_description():   

    ld = LaunchDescription()

    twist2pic_node = Node(
        package='twist2pic',
        executable='twist2pic',
        output='screen',
        parameters=[
            {'input_topic': '/target_vel'}  # Adjust the input topic if needed
        ]
    )
    ld.add_action(twist2pic_node)

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