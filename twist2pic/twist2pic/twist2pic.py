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
            Twist, "/target_vel", self.on_target_velocity, qos_profile=1
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

    def draw_drone(self, O, theta, shadow):
        drone_radius = 0.125
        drone_pix_radius = int(drone_radius * self.pixel_per_meter)

        color = (0, 0, 0)
        body_thickness = 5
        rotor_thickness = 3
        if shadow:
            color = (255, 200, 200)
            rotor_thickness = -1

        ct, st = math.cos(theta), math.sin(theta)
        r1 = self.to_pix(self.rotate((drone_radius, drone_radius), ct, st) + O)
        r2 = self.to_pix(self.rotate((drone_radius, -drone_radius), ct, st) + O)
        r3 = self.to_pix(self.rotate((-drone_radius, -drone_radius), ct, st) + O)
        r4 = self.to_pix(self.rotate((-drone_radius, drone_radius), ct, st) + O)
        a = self.to_pix(self.rotate((0, -drone_radius), ct, st) + O)
        b = self.to_pix(self.rotate((0, 2.5 * drone_radius), ct, st) + O)

        cv2.circle(self.img, r1, drone_pix_radius, color, rotor_thickness)
        cv2.circle(self.img, r2, drone_pix_radius, color, rotor_thickness)
        cv2.circle(self.img, r3, drone_pix_radius, color, rotor_thickness)
        cv2.circle(self.img, r4, drone_pix_radius, color, rotor_thickness)
        cv2.line(self.img, a, b, color, body_thickness)

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

            self.draw_drone((0, 0), 0, True)

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

            self.draw_drone(pos, theta, False)

            # Let us draw the z speed

            z = self.to_pix((0, t * self.target_velocity.linear.z))
            z_rad = 10
            z_pos = z_rad + 5
            cv2.line(self.img, (z_pos, 0), (z_pos, self.img_side), (0, 0, 200), 1)
            cv2.circle(self.img, (z_pos, z[1]), z_rad, (0, 0, 255), -1)

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