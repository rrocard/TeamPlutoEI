
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from behavior_interface.msg import BehaviorStatus
from .base_behavior import BaseBehavior
from .auto_off import AutoOff



class Takeoff(AutoOff):
    def __init__(self):
        super().__init__("Takeoff")
        self.takeoff_publisher = self.create_publisher(Empty, '/bebop/takeoff', 10)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            self.takeoff_publisher.publish(Empty())
            self.get_logger().info("Takeoff is now active and doing its task.")
        else:
            self.get_logger().info("Takeoff is now inactive.")


class Landing(AutoOff):
    def __init__(self):
        super().__init__("Landing")
        self.land_publisher = self.create_publisher(Empty, '/bebop/land', 10)

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.timer.reset()
            self.land_publisher.publish(Empty())
            self.get_logger().info("Landing is now active and doing its task.")
        else:
            self.get_logger().info("Landing is now inactive.")


# Cree un node pour chaque Behavior

def takeoff():
    rclpy.init()
    takeoff = Takeoff()
    rclpy.spin(takeoff)
    takeoff.destroy_node()
    rclpy.shutdown()

def landing():
    rclpy.init()
    landing = Landing()
    rclpy.spin(landing)
    landing.destroy_node()
    rclpy.shutdown()