#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32, Bool

class UTurn(Node):
    def __init__(self):
        super().__init__('u_turn')

        # Subscribers and Publishers
        self.status_sub = self.create_subscription(Bool, '/u_turn', self.status_callback, 10)
        self.angular_z_pub = self.create_publisher(Float32, '/angular_z', 10)

        # Parameters
        self.declare_parameter('angular_velocity', 3.0)  # rad/s
        self.declare_parameter('duration', 1.0)       # seconds
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.duration = self.get_parameter('duration').value

        self.get_logger().info("U-Turn Node has been started")

        # Internal state
        self.active = False

    def status_callback(self, msg):
        if msg.data and not self.active:
            self.active = True
            self.perform_u_turn()

    def perform_u_turn(self):
        # Create a timer to stop the U-turn after the specified duration
        self.timer = self.create_timer(self.duration, self.stop_u_turn)

        # Publish angular velocity command
        msg = Float32()
        msg.data = self.angular_velocity
        self.angular_z_pub.publish(msg)

    def stop_u_turn(self):
        # Stop the timer and reset the state
        self.timer.cancel()
        self.active = False

        # Publish zero angular velocity to stop the rotation
        msg = Float32()
        msg.data = 0.0
        self.angular_z_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    u_turn_node = UTurn()
    rclpy.spin(u_turn_node)
    u_turn_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()