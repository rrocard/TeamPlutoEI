import rclpy
from rclpy.node import Node
from behavior_interface.msg import BehaviorStatus

class BaseBehavior(Node):
    def __init__(self, behavior_name: str):
        super().__init__('base_behavior_' + behavior_name)
        self.behavior_name = behavior_name
        self.active = False
        # Subscriber to the "behavior" topic
        self.subscription = self.create_subscription(
            BehaviorStatus,
            'behavior',
            self.status_callback,
            10
        )
        # Publisher to the "behaviors_status" topic
        self.status_publisher = self.create_publisher(BehaviorStatus, 'behaviors_status', 10)

    def status_callback(self, msg: BehaviorStatus):
        # Respond to a ping or activate/deactivate based on the behavior name
        if msg.name == "ping":
            # Publish the current status if it's a ping
            self.publish_status()
        elif msg.name == self.behavior_name:
            # Update active status if the message is for this behavior
            self.active = msg.status
            self.on_status(self.active)

    def publish_status(self):
        # Create and publish a BehaviorStatus message with the current status
        status_msg = BehaviorStatus()
        status_msg.name = self.behavior_name
        status_msg.status = self.active
        self.status_publisher.publish(status_msg)

    def on_status(self, status: bool):
        self.get_logger().info(f'Behavior "{self.behavior_name}" status changed to: {"active" if status else "inactive"}')


# Fake behavior classes as before
class FakeBehaviorA(BaseBehavior):
    def __init__(self):
        super().__init__("FakeBehaviorA")

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.get_logger().info("FakeBehaviorA is now active.")
        else:
            self.get_logger().info("FakeBehaviorA is now inactive.")

class FakeBehaviorB(BaseBehavior):
    def __init__(self):
        super().__init__("FakeBehaviorB")

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.get_logger().info("FakeBehaviorB is now active.")
        else:
            self.get_logger().info("FakeBehaviorB is now inactive.")


def main():
    rclpy.init()
    # Spin nodes for both behaviors
    fake_behavior_a = FakeBehaviorA()
    fake_behavior_b = FakeBehaviorB()

    rclpy.spin(fake_behavior_a)
    rclpy.spin(fake_behavior_b)

    fake_behavior_a.destroy_node()
    fake_behavior_b.destroy_node()
    rclpy.shutdown()
