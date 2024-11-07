#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist, Pose, Point, Vector3
from nav_msgs.msg import Odometry
import math
import threading

class Slide(Node):
    def __init__(self, direction):
        super().__init__('slide')

        # Determine direction
        if direction == "left":
            self.direction_multiplier = 1.0
        elif direction == "right":
            self.direction_multiplier = -1.0
        else:
            raise ValueError("Invalid direction. Should be 'left' or 'right'")

        # Subscribers and Publishers
        self.status_sub = self.create_subscription(Bool, f'/slide_{direction}/status', self.status_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.linear_x_pub = self.create_publisher(Float32, '/linear_x', 10)
        self.angular_z_pub = self.create_publisher(Float32, '/angular_z', 10)

        # Parameters
        self.declare_parameter('open_loop_angular_velocity', 3.0)  # rad/s
        self.declare_parameter('open_loop_duration', 1.0)       # seconds
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.0)
        self.open_loop_angular_velocity = self.get_parameter('open_loop_angular_velocity').value
        self.open_loop_duration = self.get_parameter('open_loop_duration').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value

        self.get_logger().info(f"Slide {direction.capitalize()} Node has been started")

        # Internal state
        self.active = False
        self.axis_point = None
        self.axis_vector = None
        self.odom = None
        self.mutex = threading.Lock()

    def status_callback(self, msg):
        if msg.data and not self.active:
            with self.mutex:
                self.active = True
                self.slide()

    def odom_callback(self, msg):
        with self.mutex:
            self.odom = msg

    def slide(self):
        if self.odom is None:
            self.get_logger().warn("No odometry data received yet. Cannot start slide.")
            return

        # Record the axis
        self.record_axis()

        # Open loop rotation
        self.open_loop_rotate()

        # Closed loop alignment
        self.closed_loop_align()

    def record_axis(self):
        # Get current pose and orientation
        position = self.odom.pose.pose.position
        orientation = self.odom.pose.pose.orientation

        # Calculate axis vector
        qw = orientation.w
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        self.axis_vector = Vector3(
            x=qw**2 + qx**2 - qy**2 - qz**2,
            y=2 * (qw * qz + qx * qy),
            z=0.0  # Projecting onto the x-y plane
        )

        # Store axis point
        self.axis_point = Point(x=position.x, y=position.y, z=0.0)  # Projecting onto the x-y plane

    def open_loop_rotate(self):
        # Create a timer to stop the rotation after the specified duration
        self.timer = self.create_timer(self.open_loop_duration, self.stop_open_loop_rotation)

        # Publish angular velocity command
        msg = Float32()
        msg.data = self.open_loop_angular_velocity * self.direction_multiplier
        self.angular_z_pub.publish(msg)

    def stop_open_loop_rotation(self):
        # Stop the timer
        self.timer.cancel()

        # Publish zero angular velocity to stop the rotation
        msg = Float32()
        msg.data = 0.0
        self.angular_z_pub.publish(msg)

    def closed_loop_align(self):
        while self.active:
            with self.mutex:
                if self.odom is None:
                    continue

                # Calculate distance and angle to the axis
                distance, angle = self.calculate_distance_and_angle()

                # Compute and publish control commands
                linear_x_cmd = self.linear_gain * distance
                angular_z_cmd = self.angular_gain * angle

                msg_linear_x = Float32()
                msg_linear_x.data = linear_x_cmd
                self.linear_x_pub.publish(msg_linear_x)

                msg_angular_z = Float32()
                msg_angular_z.data = angular_z_cmd
                self.angular_z_pub.publish(msg_angular_z)

            # Sleep for a short duration
            rclpy.spin_once(self, timeout_sec=0.1)

        # Publish zero velocities to stop the drone after deactivation
        msg_linear_x = Float32()
        msg_linear_x.data = 0.0
        self.linear_x_pub.publish(msg_linear_x)

        msg_angular_z = Float32()
        msg_angular_z.data = 0.0
        self.angular_z_pub.publish(msg_angular_z)

    def calculate_distance_and_angle(self):
        # Get current pose and orientation
        position = self.odom.pose.pose.position
        orientation = self.odom.pose.pose.orientation

        # Calculate forward vector
        qw = orientation.w
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        forward_vector = Vector3(
            x=qw**2 + qx**2 - qy**2 - qz**2,
            y=2 * (qw * qz + qx * qy),
            z=0.0  # Projecting onto the x-y plane
        )

        # Calculate PD vector
        OD = Vector3(x=position.x - self.axis_point.x, y=position.y - self.axis_point.y, z=0.0)
        PD = OD - self.axis_vector * (self.axis_vector.x * OD.x + self.axis_vector.y * OD.y) / (math.hypot(self.axis_vector.x, self.axis_vector.y))**2

        # Calculate signed distance
        distance = math.copysign(math.hypot(PD.x, PD.y), forward_vector.x * PD.y - forward_vector.y * PD.x)

        # Calculate angle
        angle = math.atan2(forward_vector.x * self.axis_vector.y - forward_vector.y * self.axis_vector.x, forward_vector.x * self.axis_vector.x + forward_vector.y * self.axis_vector.y)

        return distance, angle

def main(args=None):
    rclpy.init(args=args)

    # You need to specify the direction ('left' or 'right') when creating the node
    slide_left_node = Slide(direction="left")  
    slide_right_node = Slide(direction="right")

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(slide_left_node)
    executor.add_node(slide_right_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        slide_left_node.destroy_node()
        slide_right_node.destroy_node()

if __name__ == '__main__':
    main()