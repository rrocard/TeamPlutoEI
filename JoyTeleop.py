import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop(Node):

    def __init__(self):
        super().__init__('joy_teleop')

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.twist_pub = self.create_publisher(Twist, '/bebop/cmd_vel', 10)

        self.last_rt_button = 0

    def joy_callback(self, msg):
        twist = Twist()

        # Roll/pitch control (left stick)
        twist.linear.y = msg.axes[0]  # Left stick horizontal (roll)
        twist.linear.x = msg.axes[1]  # Left stick vertical (pitch)

        # Yaw velocity/vertical velocity control (right stick)
        twist.angular.z = msg.axes[3]  # Right stick horizontal (yaw)
        twist.linear.z = msg.axes[4]  # Right stick vertical (vertical velocity)

        # Take-off/landing control (RT button)
        rt_button = msg.buttons[5]  
        if rt_button == 1 and self.last_rt_button == 0:
            # RT button pressed: Trigger take-off
            self.get_logger().info('Taking off') 
        elif rt_button == 0 and self.last_rt_button == 1:
            # RT button released: Trigger landing
            self.get_logger().info('Landing')
        self.last_rt_button = rt_button

        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    joy_teleop = JoyTeleop()

    rclpy.spin(joy_teleop)

    joy_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()