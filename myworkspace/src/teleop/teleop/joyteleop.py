import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy  # Import the Joy message
from std_msgs.msg import Float64

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joyteleop')

        # Declare the hover mode toggle parameter (set default to False)
        self.declare_parameter('hover_mode', False)

        # Create a publisher for the hover mode toggle
        self.hover_mode_pub = self.create_publisher(Bool, 'hover_mode_toggle', 10)

        # Create publishers for the individual control axes
        self.linear_x_pub = self.create_publisher(Float64, 'linear_x', 10)
        self.linear_y_pub = self.create_publisher(Float64, 'linear_y', 10)
        self.linear_z_pub = self.create_publisher(Float64, 'linear_z', 10)
        self.angular_z_pub = self.create_publisher(Float64, 'angular_z', 10)

        # Create a subscription to the joystick input (sensor_msgs/Joy)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Store hover mode state
        self.hover_mode = self.get_parameter('hover_mode').get_parameter_value().bool_value

    def joy_callback(self, msg):
        # Log joystick button values for debugging
        #self.get_logger().info(f"Buttons: {msg.buttons}")

        # The joystick input mapping
        # The axes and buttons are typically mapped like this:
        # axes[0] -> left stick horizontal (linear_x)
        # axes[1] -> left stick vertical (linear_y)
        # axes[2] -> right stick horizontal (linear_z)
        # axes[3] -> right stick vertical (angular_z)
        
        # Joystick control mapping with discretization
        linear_x = self.discretize_joystick_input(msg.axes[0])  # Left stick horizontal
        linear_y = self.discretize_joystick_input(msg.axes[1])  # Left stick vertical
        angular_z = self.discretize_joystick_input(msg.axes[3])  # Right stick horizontal
        linear_z = self.discretize_joystick_input(msg.axes[4])  # Right stick vertical

        
        self.get_logger().info(f"discretize test : {linear_x}")

        # Publish the joystick axes values to respective topics
        self.publish_joystick_values(linear_x, linear_y, linear_z, angular_z)

        # Hover mode toggle based on button press (e.g., button 4 to toggle hover mode)
        if msg.buttons[4] == 1:  # Button 4 toggles hover mode
            self.hover_mode = not self.hover_mode
            #self.get_logger().info(f"Hover Mode {'ON' if self.hover_mode else 'OFF'}")
            self.hover_mode_pub.publish(Bool(data=self.hover_mode))

    def discretize_joystick_input(self, value):
        if value > 0.1:
            return value  # Forward or right (depending on axis)
        elif value < -0.1:
            return value  # Backward or left (depending on axis)
        else:
            return 0.0  # No movement (neutral)

    def publish_joystick_values(self, linear_x, linear_y, linear_z, angular_z):
        # Publish the joystick inputs to the individual control topics
        self.linear_x_pub.publish(Float64(data=linear_x))
        self.linear_y_pub.publish(Float64(data=linear_y))
        self.linear_z_pub.publish(Float64(data=linear_z))
        self.angular_z_pub.publish(Float64(data=angular_z))

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
