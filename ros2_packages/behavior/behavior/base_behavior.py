import rclpy
import threading
from rclpy.node import Node
from behavior_interface.msg import BehaviorStatus


class BaseBehavior(Node):
    def __init__(self, behavior_name: str):
        super().__init__('base_behavior')
        self.behavior_name = behavior_name
        self.active = False
        self.subscription = self.create_subscription(
            BehaviorStatus,
            'behavior',
            self.status_callback,
            10
        )

        self.status_publisher = self.create_publisher(
            BehaviorStatus,
            'behavior_status',
            10
        )

    def status_callback(self, msg: BehaviorStatus):
        self.get_logger().info(f'Behavior "{self.behavior_name}"')	
        if msg.name == self.behavior_name:
            self.active = msg.status
            self.on_status(self.active)
        elif msg.name == "ping":
            reponse = BehaviorStatus()
            reponse.name = self.behavior_name
            reponse.status = self.active
            self.status_publisher.publish(reponse)
            self.get_logger().info(f'Ping received. Sending status for "{self.behavior_name}": {"active" if self.active else "inactive"}')


    def on_status(self, status: bool):
        self.get_logger().info(f'Behavior "{self.behavior_name}" status changed to: {"active" if status else "inactive"}')	



# Definit fake behaviors comme sous-classes de BaseBehavior
class FakeBehaviorA(BaseBehavior):
    def __init__(self):
        super().__init__("FakeBehaviorA")

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.get_logger().info("FakeBehaviorA is now active and doing its task.")
        else:
            self.get_logger().info("FakeBehaviorA is now inactive.")


class FakeBehaviorB(BaseBehavior):
    def __init__(self):
        super().__init__("FakeBehaviorB")

    def on_status(self, status: bool):
        super().on_status(status)
        if status:
            self.get_logger().info("FakeBehaviorB is now active and ready for action.")
        else:
            self.get_logger().info("FakeBehaviorB is now inactive.")


# Cree des nodes séparés pour chaque fakeBehavior

def fake_behavior_a():
    rclpy.init()
    fake_behavior_a = FakeBehaviorA()
    rclpy.spin(fake_behavior_a)
    fake_behavior_a.destroy_node()
    rclpy.shutdown()


def fake_behavior_b():
    rclpy.init()
    fake_behavior_b = FakeBehaviorB()
    rclpy.spin(fake_behavior_b)
    fake_behavior_b.destroy_node()
    rclpy.shutdown()
