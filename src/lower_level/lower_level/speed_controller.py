import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64  # Import the correct message type for float
from lower_level.pid import PID  # Import the PID class from pid.py file
import datetime

class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller')

        # Subscribers to the individual velocity components
        self.linear_x_sub = self.create_subscription(Float64, 'linear_x', self.linear_x_callback, 10)
        self.linear_y_sub = self.create_subscription(Float64, 'linear_y', self.linear_y_callback, 10)
        self.linear_z_sub = self.create_subscription(Float64, 'linear_z', self.linear_z_callback, 10)
        self.angular_z_sub = self.create_subscription(Float64, 'angular_z', self.angular_z_callback, 10)

        # Publisher for the target velocity
        self.target_velocity_pub = self.create_publisher(Twist, 'target_velocity', 10)

        self.get_logger().info("SpeedController Node has been started")

        # Initialize velocities to zero
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_z = 0.0

        # Create PID controllers for linear_x and linear_y
        self.pid_linear_x = PID({'Kp': 1.0, 'Kd': 0.1, 'Ki': 0.01})
        self.pid_linear_y = PID({'Kp': 1.0, 'Kd': 0.1, 'Ki': 0.01})

        # Time tracking for PID updates
        self.last_time = datetime.datetime.now()

    def linear_x_callback(self, msg: Float64):
        # Calculate the time difference for the PID update
        current_time = datetime.datetime.now()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Compute the error and update PID
        error_x = msg.data - self.linear_x
        self.pid_linear_x.update(current_time, error_x)

        # Set the computed value for linear_x
        self.linear_x = self.pid_linear_x.command/100  # Get the PID command

        self.publish_target_velocity()

    def linear_y_callback(self, msg: Float64):
        # Calculate the time difference for the PID update
        current_time = datetime.datetime.now()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Compute the error and update PID
        error_y = msg.data - self.linear_y
        self.pid_linear_y.update(current_time, error_y)

        # Set the computed value for linear_y
        self.linear_y = self.pid_linear_y.command/100  # Get the PID command

        self.publish_target_velocity()

    def linear_z_callback(self, msg: Float64):
        self.linear_z = msg.data
        self.publish_target_velocity()

    def angular_z_callback(self, msg: Float64):
        self.angular_z = msg.data
        self.publish_target_velocity()

    def publish_target_velocity(self):
        # Create a Twist message and set the components
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.linear.y = self.linear_y
        twist.linear.z = self.linear_z
        twist.angular.z = self.angular_z

        # Publish the target velocity
        self.target_velocity_pub.publish(twist)

        #self.get_logger().info(f"Published target velocity: linear_x={self.linear_x}, linear_y={self.linear_y}, linear_z={self.linear_z}, angular_z={self.angular_z}")

def main(args=None):
    rclpy.init(args=args)
    speed_controller_node = SpeedController()
    rclpy.spin(speed_controller_node)
    speed_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
