from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    speed_controller_node = Node(
        package='low_level_control',
        executable='speed_controller',
        name='speed_controller'
    )

    slide_left_node = Node(
        package='advanced_behaviors',
        executable='slide',
        name='slide_left',
        arguments=['left']  # Pass 'left' as an argument
    )

    slide_right_node = Node(
        package='advanced_behaviors',
        executable='slide',
        name='slide_right',
        arguments=['right']  # Pass 'right' as an argument
    )

    u_turn_node = Node(
        package='advanced_behaviors',
        executable='u_turn',
        name='u_turn'
    )

    joy_teleop_node = Node(
        package='teleop',
        executable='joyteleop',
        name='joyteleop'
    )

    ld.add_action(speed_controller_node)
    ld.add_action(slide_left_node)
    ld.add_action(slide_right_node)
    ld.add_action(u_turn_node)
    ld.add_action(joy_teleop_node)

    return ld