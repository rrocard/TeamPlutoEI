import rclpy
import rclpy.node

from behavior_interface.msg import Command
from sensor_msgs.msg import Joy


BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_SELECT = 7
BUTTON_LOGITECH = 8
BUTTON_CLICK_LEFT_PAD = 9
BUTTON_CLICK_RIGHT_PAD = 10

AXIS_LEFT_HORIZONTAL = 0  # Left  joystick
AXIS_LEFT_VERTICAL = 1  # Left  joystick
AXIS_LT = 2  # Left  progressive button
AXIS_RIGHT_HORIZONTAL = 3  # Right joystick
AXIS_RIGHT_VERTICAL = 4  # Right joystick
AXIS_RT = 5  # Right progressive button
AXIS_CROSS_HORIZONTAL = 6  # Cross
AXIS_CROSS_VERTICAL = 7  # Cross


class Node(rclpy.node.Node):
    def __init__(self, node_name='command'):
        super().__init__(node_name)

    # to be filled


def main(args=None):
    rclpy.init(args=args)
    teleop = Node('teleop')

    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()
