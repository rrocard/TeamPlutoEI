
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor
from example_interfaces.msg import Float32  # std_msgs.msg.Float32 is deprecated
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float64
import math

import matplotlib

matplotlib.use("Agg")


class NodeBasedController:
    def __init__(self):
        # nodes is [pos_angle, pos_offset, linear_velocity, angular_velocity]
        self.nodes = np.empty((0, 4), float)
        self.std = 0.25

    def clear_nodes(self):
        self.nodes = np.empty((0, 4), float)

    def add_node(self, position, command):
        """
        position : a tuple of (angle, offset)
        command: a tuple of (linear velocity, angular velocity)
        """
        self.nodes = np.append(self.nodes, np.array(
            [[*position, *command]]), axis=0)

    def compute_command(self, position):
        """
        Compute the velocity command as the result of a RBF
        """
        if self.nodes.shape[0] == 0:
            return 0, 0
        np_pos = np.array([*position])
        d2 = ((self.nodes[:, :2] - np_pos) ** 2).sum(axis=1)
        weights = np.exp(-d2 / (2.0 * self.std**2))  # (Nnodes, )
        weights = weights / np.linalg.norm(weights)
        resulting_command = np.dot(weights, self.nodes[:, -2:])
        return resulting_command.tolist()


class Controller(Node):
    def __init__(self):
        super().__init__("controller")

        self.nbc = NodeBasedController()
        # Here, we define our node based controller with the nodes
        # position and associated commands
        self.linear_vel = 0.1  # m/s
        self.declare_parameter(
            "linear_vel",
            self.linear_vel,
            ParameterDescriptor(description="The maximal linear velocity"),
        )
        self.angular_vel = 0.25  # rad/s
        self.declare_parameter(
            "angular_vel",
            self.angular_vel,
            ParameterDescriptor(description="The maximal angular velocity"),
        )
        self.define_nbc_nodes()
        self.linear_xpub = self.create_publisher(Float64, "linear_x", 1)
        self.angular_zpub = self.create_publisher(Float64,"angular_z",1)

        self.angle_bounds = (-0.75, 0.75)
        self.offset_bounds = (-0.75, 0.75)
        self.img_pub = self.create_publisher(
            CompressedImage, "/debug/control/image_raw/compressed", 1
        )
        self.bridge = CvBridge()
        self.vp_offset = None
        self.vp_offset_sub = self.create_subscription(
            Float32, "vp_offset", self.on_vp_offset, 1
        )
        self.vp_angle = None
        self.vp_angle_sub = self.create_subscription(
            Float32, "vp_angle", self.on_vp_angle, 1
        )

        self.add_on_set_parameters_callback(self.cb_params)

    def define_nbc_nodes(self):
        self.nbc.clear_nodes()
        # TODO: You can define your nodes here
        # as two tuples (position in S space), (linear vel, angular vel)
        self.nbc.add_node((0, 0), (0.01, 0.0))
        #if there is no angular off, but hor off we should still rotate a bit the drone to align
        self.nbc.add_node((0, 0.25), (0.0025, 0.125))
        self.nbc.add_node((0, -0.25), (0.0025, -0.125))

        #if there is angular off but no hor off, same as before
        self.nbc.add_node((0.5, 0), (0.0025, 0.125))
        # if the angular and horizontal offset are too high, the drone will only rotate on itself 
        # to find new vanishing point
        self.nbc.add_node((0.5, 0.25), (0, 0.25))
        self.nbc.add_node((0.5, -0.25), (0, 0.25))

        #angular off but no hor off
        self.nbc.add_node((-0.5, 0), (0.0025, -0.125))

        self.nbc.add_node((-0.5, 0.25), (0, -0.25))
        self.nbc.add_node((-0.5, -0.25), (0, -0.25))

    c1 = np.array([1, 0])
    c2 = np.array([1, -1])
    c3 = np.array([1, 1])
    c4 = np.array([1, -1])
    c5 = np.array([1, -1])
    c6 = np.array([0, 1])
    c7 = np.array([1, 1])
    c8 = np.array([0, -1])
    c9 = np.array([1, 1])

    def cb_params(self, data):
        for p in data:
            name = p.name
            value = p.value
            setattr(self, name, value)
        self.define_nbc_nodes()
        return SetParametersResult(successful=True)

    def on_vp_offset(self, msg):
        self.vp_offset = msg.data
        self.publish_image()
        self.publish_command()

    def on_vp_angle(self, msg):
        self.vp_angle = msg.data
        self.publish_image()
        self.publish_command()

    def publish_command(self):
        if (
            self.vp_angle is not None
            and not math.isinf(self.vp_angle)
            and self.vp_offset is not None
            and not math.isinf(self.vp_offset)
            and self.vp_offset != 500
        ):
            linear_x, angular_z = self.nbc.compute_command(
                (self.vp_angle, self.vp_offset)
            )

            msg=Float64()
            msg.data=float(linear_x)
            msg2=Float64()
            msg2.data=float(angular_z)

            self.linear_xpub.publish(msg)
            self.angular_zpub.publish(msg2)

            print("input cmd/vel",linear_x,angular_z)

        
        elif self.vp_offset == 500:
            print("no vp detected")
            msg=Float64()
            msg2=Float64()
            msg.data=float(0)
            msg2.data=float(0)

            print("input cmd/vel",msg,msg2)



    def draw_velocity(self, ax, velocity, bounds, position, xoffset, fillcolor):
        width = 0.04
        height = 0.1
        left, bottom, width, height = (
            position[0] - width / 2.0 + xoffset,
            position[1] - height / 2,
            width,
            height,
        )
        rect = mpatches.Rectangle(
            (left, bottom),
            width,
            height,
            fill=False,
            color="black",
            linewidth=2,
            alpha=0.65,
            zorder=-1,
        )
        ax.add_patch(rect)
        velocity = np.clip(velocity, bounds[0], bounds[1])
        filled_percentage = (velocity - bounds[0]) / (bounds[1] - bounds[0])
        left, bottom, width, height = (
            position[0] - width / 2.0,
            position[1] - height / 2,
            width,
            height,
        )
        left, bottom, width, height = (
            position[0] - width / 2.0 + xoffset,
            position[1] - height / 2,
            width,
            filled_percentage * height,
        )
        rect = mpatches.Rectangle(
            (left, bottom),
            width,
            height,
            fill=True,
            facecolor=fillcolor,
            alpha=0.65,
            zorder=-1,
        )
        ax.add_patch(rect)

    def publish_image(self):
        if self.img_pub.get_subscription_count() > 0:

            fig, ax = plt.subplots()
            ax.spines[["left", "bottom"]].set_position("center")
            ax.spines[["top", "right"]].set_visible(False)
            ax.set_xlabel("Angle", loc="right")
            ax.set_ylabel("Offset", loc="top")
            ax.set_xlim(*self.angle_bounds)
            ax.set_ylim(*self.offset_bounds)

            if (
                self.vp_angle is not None
                and not math.isinf(self.vp_angle)
                and self.vp_offset is not None
                and not math.isinf(self.vp_offset)
            ):
                # (x_vp_angle, x_vp_offset) is (horizontal, vertical)
                plt.plot(
                    self.vp_angle,
                    self.vp_offset,
                    marker="x",
                    markersize=20,
                    markeredgecolor="r",
                    markeredgewidth=4,
                    zorder=1,
                )

            num = 5
            for angle_i in np.linspace(self.angle_bounds[0], self.angle_bounds[1], num):
                for offset_i in np.linspace(
                    self.offset_bounds[0], self.offset_bounds[1], num
                ):
                    linear_x, angular_z = self.nbc.compute_command(
                        (angle_i, offset_i))
                    self.draw_velocity(
                        ax,
                        linear_x,
                        (0, self.linear_vel),
                        (angle_i, offset_i),
                        -0.04 / 2.0,
                        "blue",
                    )
                    self.draw_velocity(
                        ax,
                        angular_z,
                        (-self.angular_vel, self.angular_vel),
                        (angle_i, offset_i),
                        0.04 / 2.0,
                        "green",
                    )

            fig.canvas.draw()  # Draw the canvas
            img = np.frombuffer(fig.canvas.tostring_rgb(), dtype="uint8")
            img = img.reshape(*reversed(fig.canvas.get_width_height()), 3)
            # Flip RGB to BGR
            img = img[:, :, ::-1]
            self.img_pub.publish(self.bridge.cv2_to_compressed_imgmsg(img))


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()
