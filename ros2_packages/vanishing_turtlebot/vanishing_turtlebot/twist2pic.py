#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from multiprocessing import Lock
from cv_bridge import CvBridge


class Twist2Pic(Node):
    def __init__(self):
        super().__init__("twist2pic")
        self.pixel_per_meter = 100
        self.area_radius = 1.5
        self.img_center = int(self.pixel_per_meter * self.area_radius)
        self.img_side = 2 * self.img_center + 1
        self.img = np.zeros((self.img_side, self.img_side, 3), np.uint8)
        self.mutex = Lock()

        self.changed = True
        self.target_velocity = Twist()
        self.bridge = CvBridge()

        self.sub_tgt_vel = self.create_subscription(
            Twist, "cmd_vel", self.on_target_velocity, qos_profile=1
        )
        self.pub_img = self.create_publisher(
            CompressedImage, "/image_out/compressed", qos_profile=1
        )
        self.create_timer(0.1, callback=self.publish_image)

    def on_target_velocity(self, msg):
        with self.mutex:
            self.changed = True
            self.target_velocity = msg

    def rotate(self, pt, cost, sint):
        x = pt[0] * cost - pt[1] * sint
        y = pt[1] * cost + pt[0] * sint
        return np.array([x, y])

    def to_pix(self, pt):
        return (int)(self.img_center + pt[0] * self.pixel_per_meter + 0.5), (int)(
            self.img_center - pt[1] * self.pixel_per_meter + 0.5
        )

    def draw_turtlebot(self, O, theta, shadow):
        robot_radius = 0.25
        robot_pix_radius = int(robot_radius * self.pixel_per_meter)
        nose_radius = 0.04
        nose_pix_radius = int(nose_radius * self.pixel_per_meter)

        color = (0, 0, 0)
        body_thickness = 2
        wheel_thickness = 4
        if shadow:
            color = (255, 200, 200)
            # rotor_thickness = -1

        ct, st = math.cos(theta), math.sin(theta)
        rnose = self.to_pix(self.rotate((0, robot_radius), ct, st) + O)
        right_wheel_a = self.to_pix(
            self.rotate((robot_radius, -robot_radius / 2.0), ct, st) + O
        )
        right_wheel_b = self.to_pix(
            self.rotate((robot_radius, robot_radius / 2.0), ct, st) + O
        )
        left_wheel_a = self.to_pix(
            self.rotate((-robot_radius, -robot_radius / 2.0), ct, st) + O
        )
        left_wheel_b = self.to_pix(
            self.rotate((-robot_radius, robot_radius / 2.0), ct, st) + O
        )

        cv2.line(self.img, left_wheel_a, left_wheel_b, color, wheel_thickness)
        cv2.line(self.img, right_wheel_a, right_wheel_b, color, wheel_thickness)
        robot_center = self.to_pix(O)
        cv2.circle(self.img, robot_center, robot_pix_radius, color, body_thickness)
        cv2.circle(self.img, rnose, nose_pix_radius, color, -1)

    def draw(self):
        with self.mutex:
            if not self.changed:
                return
            self.changed = False

            self.img[...] = 255  # clear

            # Draw the grid

            val = 0
            thickness = 2
            while val <= self.area_radius:
                pix = (int)(val * self.pixel_per_meter + 0.5)
                cv2.line(
                    self.img,
                    (0, self.img_center + pix),
                    (self.img_side, self.img_center + pix),
                    (255, 200, 200),
                    thickness,
                )
                cv2.line(
                    self.img,
                    (0, self.img_center - pix),
                    (self.img_side, self.img_center - pix),
                    (255, 200, 200),
                    thickness,
                )
                cv2.line(
                    self.img,
                    (self.img_center + pix, 0),
                    (self.img_center + pix, self.img_side),
                    (255, 200, 200),
                    thickness,
                )
                cv2.line(
                    self.img,
                    (self.img_center - pix, 0),
                    (self.img_center - pix, self.img_side),
                    (255, 200, 200),
                    thickness,
                )
                val += 0.5
                thickness = 3 - thickness

            self.draw_turtlebot((0, 0), 0, True)

            # Let us simulate the motion during t second.

            t = 0.5
            nb_steps = 5
            dt = t / float(nb_steps)

            theta = 0
            pos = np.array([0.0, 0.0])
            X = np.array([0.0, 1.0])
            Y = np.array([-1.0, 0.0])

            for step in range(nb_steps):
                theta += self.target_velocity.angular.z * dt
                ct, st = math.cos(theta), math.sin(theta)
                lx = self.rotate(X, ct, st)
                ly = self.rotate(Y, ct, st)
                pos = (
                    pos
                    + (
                        lx * self.target_velocity.linear.x
                        + ly * self.target_velocity.linear.y
                    )
                    * dt
                )

            self.draw_turtlebot(pos, theta, False)

    def publish_image(self):
        if self.pub_img.get_subscription_count() > 0:
            self.draw()
            msg = self.bridge.cv2_to_compressed_imgmsg(self.img)
            self.pub_img.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    twist2pic_node = Twist2Pic()
    rclpy.spin(twist2pic_node)  # This is a blocking function

    twist2pic_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
