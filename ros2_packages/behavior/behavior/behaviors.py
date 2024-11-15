
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from behavior_interface.msg import BehaviorStatus
from .base_behavior import BaseBehavior
from .auto_off import AutoOff
import time


class TakeOff(AutoOff):
    def __init__(self):
        super().__init__("TakeOff")
        self.takeoff_publisher = self.create_publisher(
            Empty, '/bebop/takeoff', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            self.takeoff_publisher.publish(Empty())
            self.get_logger().info("TakeOff is now active and doing its task.")
        else:
            self.get_logger().info("TakeOff is now inactive.")


class Land(AutoOff):
    def __init__(self):
        super().__init__("Land")
        self.land_publisher = self.create_publisher(Empty, '/bebop/land', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            self.land_publisher.publish(Empty())
            self.get_logger().info("Land is now active and doing its task.")
        else:
            self.get_logger().info("Land is now inactive.")


class Hover(AutoOff):
    def __init__(self):
        super().__init__("Hover")
        self.hover_publisher = self.create_publisher(Bool, '/hover_mode_toggle', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            hovermsg = Bool()
            hovermsg.data = True
            self.hover_publisher.publish(hovermsg)

            self.get_logger().info("Hover is now active and doing its task.")
        else:
            self.get_logger().info("Hover is now inactive.")


class MoveForward(AutoOff):
    def __init__(self):
        super().__init__("MoveForward")
        self.moveforward_publisher = self.create_publisher(Float64, 'linear_x', 100)
        
    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            vitesse = Float64()
            vitesse.data = 0.1
            self.moveforward_publisher.publish(vitesse)
            self.get_logger().info("MoveForward is now active and doing its task.")
        else:
            self.get_logger().info("MoveForward is now inactive.")

class Center(AutoOff):
    def __init__(self):
        super().__init__("Center")
        self.x_publisher = self.create_publisher(Float64, 'linear_x', 100)
        self.y_publisher = self.create_publisher(Float64, 'linear_y', 100)
        self.z_publisher = self.create_publisher(Float64, 'linear_z', 100)
        self.az_publisher = self.create_publisher(Float64, 'angular_z', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            vitesse = Float64()
            vitesse.data = 0.0
            self.x_publisher.publish(vitesse)
            self.y_publisher.publish(vitesse)
            self.z_publisher.publish(vitesse)
            self.az_publisher.publish(vitesse)
            self.get_logger().info("Center is now active and doing its task.")
        else:
            self.get_logger().info("Center is now inactive.")


class MoveBackward(AutoOff):
    def __init__(self):
        super().__init__("MoveBackward")
        self.movebackward_publisher = self.create_publisher(Float64, 'linear_x', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            vitesse = Float64()
            vitesse.data = -0.1
            self.movebackward_publisher.publish(vitesse)
            self.get_logger().info("MoveBackward is now active and doing its task.")
        else:
            self.get_logger().info("MoveBackward is now inactive.")


class MoveRight(AutoOff):
    def __init__(self):
        super().__init__("MoveRight")
        self.moveright_publisher = self.create_publisher(Float64, 'linear_y', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            vitesse = Float64()
            vitesse.data = 0.1
            self.moveright_publisher.publish(vitesse)
            self.get_logger().info("MoveRight is now active and doing its task.")
        else:
            self.get_logger().info("MoveRight is now inactive.")


class MoveLeft(AutoOff):
    def __init__(self):
        super().__init__("MoveLeft")
        self.moveleft_publisher = self.create_publisher(Float64, 'linear_y', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            vitesse = Float64()
            vitesse.data = -0.1
            self.moveleft_publisher.publish(vitesse)
            self.get_logger().info("MoveLeft is now active and doing its task.")
        else:
            self.get_logger().info("MoveLeft is now inactive.")


class MoveUp(AutoOff):
    def __init__(self):
        super().__init__("MoveUp")
        self.moveup_publisher = self.create_publisher(Float64, 'linear_z', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            vitesse = Float64()
            vitesse.data = 0.1
            self.moveup_publisher.publish(vitesse)
            self.get_logger().info("MoveUp is now active and doing its task.")
        else:
            self.get_logger().info("MoveUp is now inactive.")


class MoveDown(AutoOff):
    def __init__(self):
        super().__init__("MoveDown")
        self.movedown_publisher = self.create_publisher(Float64, 'linear_z', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            vitesse = Float64()
            vitesse.data = -0.1
            self.movedown_publisher.publish(vitesse)
            self.get_logger().info("MoveDown is now active and doing its task.")
        else:
            self.get_logger().info("MoveDown is now inactive.")


class TurnRight(AutoOff):
    def __init__(self):
        super().__init__("TurnRight")
        self.turnright_publisher = self.create_publisher(
            Float64, 'angular_z', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            vitesse = Float64()
            vitesse.data = 0.1
            self.turnright_publisher.publish(vitesse)
            self.get_logger().info("TurnRight is now active and doing its task.")
        else:
            self.get_logger().info("TurnRight is now inactive.")


class TurnLeft(AutoOff):
    def __init__(self):
        super().__init__("TurnLeft")
        self.turnleft_publisher = self.create_publisher(Float64, 'angular_z', 100)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            vitesse = Float64()
            vitesse.data = -0.1
            self.turnleft_publisher.publish(vitesse)
            self.get_logger().info("TurnLeft is now active and doing its task.")
        else:
            self.get_logger().info("TurnLeft is now inactive.")


# Cree un node pour chaque Behavior

def takeoff():
    rclpy.init()
    takeoff = TakeOff()
    rclpy.spin(takeoff)
    takeoff.destroy_node()
    rclpy.shutdown()


def land():
    rclpy.init()
    land = Land()
    rclpy.spin(land)
    land.destroy_node()
    rclpy.shutdown()


def hover():
    rclpy.init()
    hover = Hover()
    rclpy.spin(hover)
    hover.destroy_node()
    rclpy.shutdown()


def center():
    rclpy.init()
    center = Center()
    rclpy.spin(center)
    center.destroy_node()
    rclpy.shutdown()

def move_forward():
    rclpy.init()
    move_forward = MoveForward()
    rclpy.spin(move_forward)
    move_forward.destroy_node()
    rclpy.shutdown()


def move_backward():
    rclpy.init()
    move_backward = MoveBackward()
    rclpy.spin(move_backward)
    move_backward.destroy_node()
    rclpy.shutdown()


def move_right():
    rclpy.init()
    move_right = MoveRight()
    rclpy.spin(move_right)
    move_right.destroy_node()
    rclpy.shutdown()


def move_left():
    rclpy.init()
    move_left = MoveLeft()
    rclpy.spin(move_left)
    move_left.destroy_node()
    rclpy.shutdown()


def move_up():
    rclpy.init()
    move_up = MoveUp()
    rclpy.spin(move_up)
    move_up.destroy_node()
    rclpy.shutdown()


def move_down():
    rclpy.init()
    move_down = MoveDown()
    rclpy.spin(move_down)
    move_down.destroy_node()
    rclpy.shutdown()


def turn_right():
    rclpy.init()
    turn_right = TurnRight()
    rclpy.spin(turn_right)
    turn_right.destroy_node()
    rclpy.shutdown()


def turn_left():
    rclpy.init()
    turn_left = TurnLeft()
    rclpy.spin(turn_left)
    turn_left.destroy_node()
    rclpy.shutdown()
