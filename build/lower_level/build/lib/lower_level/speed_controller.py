import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from lower_level.pid import PID  # Assuming you have a PID class
import time

class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller')

        # Declare parameters for PID
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.01)

        # Create the PID controllers for linear_x and linear_y
        pid_gains = {
            'Kp': self.get_parameter('kp').get_parameter_value().double_value,
            'Ki': self.get_parameter('ki').get_parameter_value().double_value,
            'Kd': self.get_parameter('kd').get_parameter_value().double_value
        }

        self.pid_linear_x = PID(pid_gains)
        self.pid_linear_y = PID(pid_gains)

        # Initialize joystick values
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_z = 0.0

        # Initialize hover mode as False (drone is not hovering by default)
        self.hover_mode = False

        # Subscribe to individual topics
        self.create_subscription(Float64, 'linear_x', self.linear_x_callback, 10)
        self.create_subscription(Float64, 'linear_y', self.linear_y_callback, 10)
        self.create_subscription(Float64, 'linear_z', self.linear_z_callback, 10)
        self.create_subscription(Float64, 'angular_z', self.angular_z_callback, 10)

        # Subscribe to hover mode toggle topic
        self.create_subscription(Bool, 'hover_mode_toggle', self.hover_mode_callback, 10)

        # Publisher for modified cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def linear_x_callback(self, msg):
        self.linear_x = msg.data
        #self.get_logger().info(f"Received linear_x: {self.linear_x}")

    def linear_y_callback(self, msg):
        self.linear_y = msg.data
        #self.get_logger().info(f"Received linear_y: {self.linear_y}")

    def linear_z_callback(self, msg):
        self.linear_z = msg.data
        #self.get_logger().info(f"Received linear_z: {self.linear_z}")

    def angular_z_callback(self, msg):
        self.angular_z = msg.data
        #self.get_logger().info(f"Received angular_z: {self.angular_z}")

    def hover_mode_callback(self, msg):
        # Toggle hover mode based on received message
        self.hover_mode = msg.data
        if self.hover_mode:
            self.get_logger().info("Hover Mode ON: Stopping drone.")
        else:
            self.get_logger().info("Hover Mode OFF: Drone moving with joystick inputs.")

    def publish_cmd_vel(self):
        # Log the current joystick values
        self.get_logger().info(f"Current joystick values -> linear_x: {self.linear_x}, linear_y: {self.linear_y}, angular_z: {self.angular_z}, linear_z: {self.linear_z}")

        # Apply PID if hover mode is OFF, otherwise set linear speeds to zero
        if self.hover_mode:
            # Stop the drone if hover mode is ON
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
            self.linear_z = 0.0
            self.pid_linear_x.reset()
            self.pid_linear_y.reset()
            #self.get_logger().info("Hover mode is ON, stopping drone.")
        else:
            # Apply PID control to joystick input if hover mode is OFF
            self.pid_linear_x.update(time.time(), self.linear_x)
            self.pid_linear_y.update(time.time(), self.linear_y)
            self.linear_x = self.pid_linear_x.command  # Apply PID controlled values
            self.linear_y = self.pid_linear_y.command  # Apply PID controlled values

            #self.get_logger().info(f"PID updated values -> linear_x: {self.linear_x}, linear_y: {self.linear_y}")

        # Construct the new Twist message to send to the drone
        new_msg = Twist()
        new_msg.linear.x = self.linear_x
        new_msg.linear.y = self.linear_y
        new_msg.linear.z = self.linear_z  # Ensure linear_z is correctly set
        new_msg.angular.z = self.angular_z

        # Log the Twist message that will be published
        self.get_logger().info(f"Publishing cmd_vel -> linear_x: {new_msg.linear.x}, linear_y: {new_msg.linear.y}, linear_z: {new_msg.linear.z}, angular_z: {new_msg.angular.z}")

        # Publish the modified command to 'cmd_vel'
        self.cmd_vel_pub.publish(new_msg)

    def timer_callback(self):
        # This method is periodically called to publish cmd_vel at a fixed rate
        self.publish_cmd_vel()


def main(args=None):
    rclpy.init(args=args)
    node = SpeedController()

    # Set up a timer to periodically call publish_cmd_vel() at a fixed rate
    timer_period = 0.1  # Publish at 10Hz
    node.create_timer(timer_period, node.timer_callback)

    # Spin the node to keep it alive and handle callbacks
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
