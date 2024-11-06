import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # Import the correct message type for single floats
from sensor_msgs.msg import Joy

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        # Publisher for each velocity component
        self.linear_x_pub = self.create_publisher(Float64, 'linear_x', 10)
        self.linear_y_pub = self.create_publisher(Float64, 'linear_y', 10)
        self.linear_z_pub = self.create_publisher(Float64, 'linear_z', 10)
        self.angular_z_pub = self.create_publisher(Float64, 'angular_z', 10)

        # Subscribe to the joystick messages (sensor_msgs/Joy)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info("JoyTeleop Node has been started")

    def joy_callback(self, msg: Joy):
        # Assuming axes 0, 1, 2 for linear_x, linear_y, linear_z, and axis 3 for angular_z
        linear_x = self.discretize(msg.axes[0])  # Left stick X (linear x)
        linear_y = self.discretize(msg.axes[1])  # Left stick Y (linear y)
        linear_z = self.discretize(msg.axes[2])  # Left stick Z (linear z)
        angular_z = self.discretize(msg.axes[3])  # Right stick X (angular z)

        # Publish the velocity components to the respective topics
        self.linear_x_pub.publish(Float64(data=linear_x))
        self.linear_y_pub.publish(Float64(data=linear_y))
        self.linear_z_pub.publish(Float64(data=linear_z))
        self.angular_z_pub.publish(Float64(data=angular_z))

        #self.get_logger().info(f"Published velocities: linear_x={linear_x}, linear_y={linear_y}, linear_z={linear_z}, angular_z={angular_z}")

    def discretize(self, value: float) -> float:
        """Discretize the joystick input: set it to 0 if it's between -0.2 and 0.2, otherwise to -1 or 1."""
        if -0.2 < value < 0.2:
            return 0.0
        elif value > 0:
            return 1.0
        else:
            return -1.0

def main(args=None):
    rclpy.init(args=args)
    joy_teleop_node = JoyTeleop()
    rclpy.spin(joy_teleop_node)
    joy_teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
