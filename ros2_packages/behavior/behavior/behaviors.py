
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from behavior_interface.msg import BehaviorStatus
from .base_behavior import BaseBehavior
from .auto_off import AutoOff



class TakeOff(AutoOff):
    def __init__(self):
        super().__init__("TakeOff")
        self.takeoff_publisher = self.create_publisher(Empty, '/bebop/takeoff', 10)

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
        self.land_publisher = self.create_publisher(Empty, '/bebop/land', 10)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            self.land_publisher.publish(Empty())
            self.get_logger().info("Land is now active and doing its task.")
        else:
            self.get_logger().info("Land is now inactive.")

class Hover(BaseBehavior):
    def __init__(self):
        super().__init__("Hover")
        self.hover_publisher = self.create_publisher(Empty, 'hover', 10)   #SÛR QUE HOVER ?

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.hover_publisher.publish(Empty())
            self.get_logger().info("Hover is now active and doing its task.")
        else:
            self.get_logger().info("Hover is now inactive.")

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