#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class SpeedController(Node):

    def __init__(self):
        super().__init__('speed_controller')

        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscribers for the individual velocity topics
        self.linear_x_sub = self.create_subscription(Float32, '/linear_x', self.linear_x_callback, 10)
        self.linear_y_sub = self.create_subscription(Float32, '/linear_y', self.linear_y_callback, 10)
        self.linear_z_sub = self.create_subscription(Float32, '/linear_z', self.linear_z_callback, 10)
        self.angular_z_sub = self.create_subscription(Float32, '/angular_z', self.angular_z_callback, 10)

        # Initialize target velocities
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_linear_z = 0.0
        self.target_angular_z = 0.0

        # Create a timer to run the control loop periodically
        self.timer = self.create_timer(0.1, self.control_loop)  # Control loop at 10 Hz

    def linear_x_callback(self, msg):
        self.target_linear_x = msg.data

    def linear_y_callback(self, msg):
        self.target_linear_y = msg.data

    def linear_z_callback(self, msg):
        self.target_linear_z = msg.data

    def angular_z_callback(self, msg):
        self.target_angular_z = msg.data

    def control_loop(self):
        # Create the Twist message
        cmd_vel_msg = Twist()

        # Set the values of the Twist message based on the target velocities
        cmd_vel_msg.linear.x = self.target_linear_x
        cmd_vel_msg.linear.y = self.target_linear_y
        cmd_vel_msg.linear.z = self.target_linear_z
        cmd_vel_msg.angular.z = self.target_angular_z

        # Publish the cmd_vel message
        self.cmd_vel_pub.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)

    speed_controller = SpeedController()

    rclpy.spin(speed_controller)

    speed_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
