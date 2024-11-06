import rclpy
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
            'behaviors_status',
            10
        )

    def status_callback(self, msg: BehaviorStatus):
        if msg.name == self.behavior_name:
            self.active = msg.status
            self.on_status(self.active)
        elif msg.name == "ping":
            reponse = BehaviorStatus()
            reponse.name = self.behavior_name
            reponse.status = self.active
            self.status_publisher.publish(reponse)

    def on_status(self, status: bool):
        self.get_logger().info(
            f'Behavior "{self.behavior_name}" status changed to: {"active" if status else "inactive"}')


# Definit fake behavior comme sous-classes de BaseBehavior


# class Takeoff(BaseBehavior):
#     def __init__(self):
#         super().__init__("Takeoff")

#     def on_status(self, status: bool):
#         super().on_status(status)
#         if status:
#             self.get_logger().info("Takeoff is now active and doing its task.")
#         else:
#             self.get_logger().info("Takeoff is now inactive.")


# class Landing(BaseBehavior):
#     def __init__(self):
#         super().__init__("Landing")

#     def on_status(self, status: bool):
#         super().on_status(status)
#         if status:
#             self.get_logger().info("Landing is now active and doing its task.")
#         else:
#             self.get_logger().info("Landing is now inactive.")


# # Cree un node pour chaque Behavior

# def takeoff():
#     rclpy.init()
#     takeoff = Takeoff()
#     rclpy.spin(takeoff)
#     takeoff.destroy_node()
#     rclpy.shutdown()

# def landing():
#     rclpy.init()
#     landing = Landing()
#     rclpy.spin(landing)
#     landing.destroy_node()
#     rclpy.shutdown()




