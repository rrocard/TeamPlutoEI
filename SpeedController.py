#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from threading import Lock

class SpeedController(Node):

    def __init__(self):
        super().__init__('speed_controller')

        # Initialize target velocity
        self.target_velocity = Twist()

        # Create mutex for thread safety
        self.mutex = Lock()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/bebop/cmd_vel', 10)
        self.target_vel_pub = self.create_publisher(Twist, '/target_vel', 10) 

        # Subscribers
        self.create_subscription(Float32, '/linear_x', self.linear_x_callback, 10)
        self.create_subscription(Float32, '/linear_y', self.linear_y_callback, 10)
        self.create_subscription(Float32, '/linear_z', self.linear_z_callback, 10)
        self.create_subscription(Float32, '/angular_z', self.angular_z_callback, 10)

        # Control loop timer (adjust frequency as needed)
        self.create_timer(0.1, self.control_loop)  # 10Hz

    def linear_x_callback(self, msg):
        with self.mutex:
            self.target_velocity.linear.x = msg.data

    def linear_y_callback(self, msg):
        with self.mutex:
            self.target_velocity.linear.y = msg.data

    def linear_z_callback(self, msg):
        with self.mutex:
            self.target_velocity.linear.z = msg.data

    def angular_z_callback(self, msg):
        with self.mutex:
            self.target_velocity.angular.z = msg.data

    def control_loop(self):
        with self.mutex:
            # TODO: Implement PID controllers for roll/pitch here
            # For now, directly use target velocities for simplicity
            cmd_vel = Twist()
            cmd_vel.linear.x = self.target_velocity.linear.x  # Pitch
            cmd_vel.linear.y = self.target_velocity.linear.y  # Roll
            cmd_vel.linear.z = self.target_velocity.linear.z
            cmd_vel.angular.z = self.target_velocity.angular.z

            # Publish commands
            self.cmd_vel_pub.publish(cmd_vel)
            self.target_vel_pub.publish(self.target_velocity)


def main(args=None):
    rclpy.init(args=args)

    speed_controller = SpeedController()

    rclpy.spin(speed_controller)

    speed_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()